import pandas as pd
import numpy as np
from scipy.ndimage import gaussian_filter1d
import json


class CMJ_Analyser:
    @staticmethod
    def compute_cmj_test(input_data):
        """
        Perform CMJ test calculations.

        Args:
            input_data (dict): JSON input containing `left_force`, `right_force`, `total_force`, and `index`.

        Returns:
            dict: Combined results including key points and phase areas.
        """
        # Extract and validate data
        try:
            series_data = {
                "left_force": input_data["left_force"],
                "right_force": input_data["right_force"],
                "total_force": input_data["total_force"],
                "time": input_data["index"]
            }
        except KeyError as e:
            raise ValueError(f"Missing required data: {e}")

        # Convert data to a DataFrame
        df = pd.DataFrame(series_data)

        if 'total_force' not in df.columns or 'left_force' not in df.columns or 'right_force' not in df.columns:
            raise ValueError("The input data must contain 'total_force', 'left_force', and 'right_force' columns.")

        smoothing_sigma = 10
        df['smoothed_left'] = gaussian_filter1d(df['left_force'], sigma=smoothing_sigma)
        df['smoothed_right'] = gaussian_filter1d(df['right_force'], sigma=smoothing_sigma)
        df['smoothed_total'] = gaussian_filter1d(df['total_force'], sigma=smoothing_sigma)

        # Highest point
        highest_idx = df['smoothed_total'].idxmax()
        highest_value = df['smoothed_total'].iloc[highest_idx]
        highest_time = df['time'].iloc[highest_idx]

        # Identify peaks and valleys
        window_size = 50
        df['rolling_max'] = df['smoothed_total'].rolling(window=window_size, center=True).max()
        df['rolling_min'] = df['smoothed_total'].rolling(window=window_size, center=True).min()

        peaks = df[df['smoothed_total'] == df['rolling_max']]
        valleys = df[df['smoothed_total'] == df['rolling_min']]
        important_points = pd.concat([peaks, valleys]).sort_index()

        # Estimate bodyweight from start of signal (standing phase) for relative threshold
        n_est = min(max(50, len(df) // 10), len(df))
        bodyweight_est = df['smoothed_total'].iloc[:n_est].mean()
        # Flight = force below this; use 5% of bodyweight so plate noise / residual doesn't need to hit 2 N
        threshold = max(2.0, 0.05 * float(bodyweight_est))

        below_threshold_idx = df[df['smoothed_total'] < threshold].index.min()
        if pd.isna(below_threshold_idx):
            # No point below threshold: use end of trace so all peaks/valleys are "pre-threshold"
            below_threshold_idx = df.index.max()

        pre_threshold_points = important_points[important_points.index < below_threshold_idx]

        if not pre_threshold_points.empty:
            lowest_point = pre_threshold_points.nsmallest(1, 'smoothed_total')
            highest_points = pre_threshold_points.nlargest(2, 'smoothed_total')
            selected_points = pd.concat([lowest_point, highest_points]).sort_index()
        else:
            selected_points = pd.DataFrame()

        # Gradient for unload point
        df['gradient'] = np.gradient(df['smoothed_total'], df['time'])
        gradient_threshold = -80
        unload_idx = None
        for idx in range(1, len(df)):
            if df['gradient'].iloc[idx] < gradient_threshold:
                unload_idx = idx
                break

        if unload_idx is None:
            raise ValueError("No unload point detected. Cannot proceed.")

        unload_value = df['smoothed_total'].iloc[unload_idx]

        # Braking point
        if not selected_points.empty:
            lowest_idx = selected_points.index[0]
            braking_idx = None
            braking_time = None
            for idx in range(lowest_idx + 1, len(df)):
                if (df['smoothed_total'].iloc[idx - 1] - unload_value) * (df['smoothed_total'].iloc[idx] - unload_value) < 0:
                    x1, y1 = df['time'].iloc[idx - 1], df['smoothed_total'].iloc[idx - 1]
                    x2, y2 = df['time'].iloc[idx], df['smoothed_total'].iloc[idx]
                    braking_time = x1 + (unload_value - y1)*(x2 - x1)/(y2 - y1)
                    braking_idx = idx
                    break
        else:
            braking_idx = None
            braking_time = None

        # Takeoff & Landing
        below_threshold_points = df[df['smoothed_total'] < threshold]
        if not below_threshold_points.empty:
            takeoff_idx = below_threshold_points.index.min()
            landing_idx = below_threshold_points.index.max()
            takeoff_time = df['time'].iloc[takeoff_idx]
            takeoff_value = df['smoothed_total'].iloc[takeoff_idx]
            landing_time = df['time'].iloc[landing_idx]
            landing_value = df['smoothed_total'].iloc[landing_idx]
        else:
            takeoff_idx = takeoff_time = takeoff_value = None
            landing_idx = landing_time = landing_value = None

        # Bodyweight
        bodyweight = df['smoothed_total'].iloc[:unload_idx].mean()

        # Extract indexes for Start of Yield, Zero Velocity, Lifting Up from selected_points
        if not selected_points.empty:
            start_of_yield_idx = selected_points.index[0] if len(selected_points) > 0 else None
            zero_velocity_idx = selected_points.index[1] if len(selected_points) > 1 else None
            lifting_up_idx = selected_points.index[2] if len(selected_points) > 2 else None
        else:
            start_of_yield_idx = zero_velocity_idx = lifting_up_idx = None

        # Helper to extract point data
        def get_point(name, idx=None, time_val=None, force_val=None):
            if idx is not None and idx in df.index:
                return {"point_type": name, "time": float(df['time'].iloc[idx]), "total_force": float(df['smoothed_total'].iloc[idx])}
            elif time_val is not None and force_val is not None:
                return {"point_type": name, "time": float(time_val), "total_force": float(force_val)}
            else:
                return {"point_type": name, "time": None, "total_force": None}

        requested_points = []
        requested_points.append(get_point("Start of Unload", idx=unload_idx))
        requested_points.append(get_point("Start of Yield", idx=start_of_yield_idx))

        if braking_time is not None:
            requested_points.append(get_point("Braking Point", time_val=braking_time, force_val=unload_value))
        else:
            requested_points.append(get_point("Braking Point"))

        requested_points.append(get_point("Zero Velocity Point", idx=zero_velocity_idx))
        requested_points.append(get_point("Lifting Up", idx=lifting_up_idx))

        if takeoff_idx is not None:
            requested_points.append(get_point("Takeoff Point", idx=takeoff_idx))
        else:
            requested_points.append(get_point("Takeoff Point"))

        if landing_idx is not None:
            requested_points.append(get_point("Landing Point", idx=landing_idx))
        else:
            requested_points.append(get_point("Landing Point"))

        # Define areas (phases)
        areas_list = []
        def add_area(name, start_t, end_t):
            if start_t is not None and end_t is not None and start_t < end_t:
                areas_list.append({"phase": name, "start_time": float(start_t), "end_time": float(end_t)})

        if unload_idx is not None and start_of_yield_idx is not None:
            add_area("Unweighting Phase", df['time'].iloc[unload_idx], df['time'].iloc[start_of_yield_idx])

        if braking_idx is not None and start_of_yield_idx is not None and braking_time is not None:
            add_area("Yielding Phase", df['time'].iloc[start_of_yield_idx], braking_time)

        if braking_idx is not None and zero_velocity_idx is not None and braking_time is not None:
            add_area("Braking Phase", braking_time, df['time'].iloc[zero_velocity_idx])

        if braking_idx is not None and zero_velocity_idx is not None and start_of_yield_idx is not None:
            add_area("Eccentric Phase", df['time'].iloc[start_of_yield_idx], df['time'].iloc[zero_velocity_idx])

        if zero_velocity_idx is not None and takeoff_idx is not None:
            add_area("Concentric Phase", df['time'].iloc[zero_velocity_idx], df['time'].iloc[takeoff_idx])

        if takeoff_idx is not None and landing_idx is not None:
            add_area("Flight Phase", df['time'].iloc[takeoff_idx], df['time'].iloc[landing_idx])


        #calculating the statistics

        # Gradient for unload point
        df['gradient'] = np.gradient(df['smoothed_total'], df['time'])
        gradient_threshold = -80
        unload_idx = None
        for idx in range(1, len(df)):
            if df['gradient'].iloc[idx] < gradient_threshold:
                unload_idx = idx
                break

        if unload_idx is None:
            raise ValueError("No unload point detected. Cannot proceed.")

        unload_value = df['smoothed_total'].iloc[unload_idx]

        # Braking point
        if not selected_points.empty:
            lowest_idx = selected_points.index[0]
            braking_idx = None
            braking_time = None
            for idx in range(lowest_idx + 1, len(df)):
                if (df['smoothed_total'].iloc[idx - 1] - unload_value) * (df['smoothed_total'].iloc[idx] - unload_value) < 0:
                    x1, y1 = df['time'].iloc[idx - 1], df['smoothed_total'].iloc[idx - 1]
                    x2, y2 = df['time'].iloc[idx], df['smoothed_total'].iloc[idx]
                    braking_time = x1 + (unload_value - y1)*(x2 - x1)/(y2 - y1)
                    braking_idx = idx
                    break
        else:
            braking_idx = None
            braking_time = None

        # Takeoff & Landing
        below_threshold_points = df[df['smoothed_total'] < threshold]
        if not below_threshold_points.empty:
            takeoff_idx = below_threshold_points.index.min()
            landing_idx = below_threshold_points.index.max()
            takeoff_time = df['time'].iloc[takeoff_idx]
            takeoff_value = df['smoothed_total'].iloc[takeoff_idx]
            landing_time = df['time'].iloc[landing_idx]
            landing_value = df['smoothed_total'].iloc[landing_idx]
        else:
            takeoff_idx = takeoff_time = takeoff_value = None
            landing_idx = landing_time = landing_value = None

        # Bodyweight
        bodyweight = df['smoothed_total'].iloc[:unload_idx].mean()
        g = 9.81  # Acceleration due to gravity (m/s^2)

        # Calculate additional statistics
        impulse = net_impulse = velocity = jump_height = None
        peak_force = highest_value
        time_to_peak_force = highest_time - df['time'].iloc[unload_idx] if unload_idx is not None else None
        rfd = None
        eccentric_duration = None
        concentric_duration = None
        flight_time = None
        average_power = None

        if takeoff_idx is not None and landing_idx is not None:
            # Use force range between Start of Yield to Takeoff
            start_time = df['time'].iloc[unload_idx] if unload_idx is not None else None
            end_time = df['time'].iloc[takeoff_idx]
            time_range = end_time - start_time
            force_range = df['smoothed_total'].iloc[unload_idx:takeoff_idx+1]
            time_range_values = df['time'].iloc[unload_idx:takeoff_idx+1]

            # Impulse and derived metrics
            impulse = np.trapezoid(force_range, time_range_values)
            bodyweight_force = bodyweight * time_range
            net_impulse = impulse - bodyweight_force
            mass = bodyweight / g
            velocity = net_impulse / mass
            jump_height = (velocity ** 2) / (2 * g)

            # Rate of Force Development (RFD)
            if start_time is not None and time_to_peak_force is not None:
                rfd = (peak_force - bodyweight) / time_to_peak_force

            # Durations
            eccentric_duration = df['time'].iloc[braking_idx] - start_time if braking_idx is not None else None
            concentric_duration = takeoff_time - df['time'].iloc[braking_idx] if braking_idx is not None else None
            flight_time = landing_time - takeoff_time if takeoff_time and landing_time else None

            # Average Power
            if velocity and jump_height:
                average_power = (mass * g * velocity + (0.5 * mass * velocity ** 2)) / concentric_duration if concentric_duration else None

        # Combined data
        combined_data = {
            "points": requested_points,
            "areas": areas_list,
            "analysis": {
                "impulse": float(impulse) if impulse is not None else None,
                "net_impulse": float(net_impulse) if net_impulse is not None else None,
                "velocity": float(velocity) if velocity is not None else None,
                "jump_height": float(jump_height) if jump_height is not None else None,
                "peak_force": float(peak_force) if peak_force is not None else None,
                "time_to_peak_force": float(time_to_peak_force) if time_to_peak_force is not None else None,
                "rfd": float(rfd) if rfd is not None else None,
                "eccentric_duration": float(eccentric_duration) if eccentric_duration is not None else None,
                "concentric_duration": float(concentric_duration) if concentric_duration is not None else None,
                "flight_time": float(flight_time) if flight_time is not None else None,
                "average_power": float(average_power) if average_power is not None else None
            }
        }

        return combined_data
