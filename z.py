from itertools import combinations

LB_TO_KG = 0.45359237

def parse_plate_tokens(tokens, default_unit="kg"):
    """
    tokens examples:
      ["20kg", "20kg", "10kg", "45lb", "2.5kg"]
      ["20kg", "10kg", "45lb", "5"]  -> uses default_unit for bare numbers
    Returns: list of (value_kg, label)
    """
    plates = []
    for t in tokens:
        s = str(t).strip().lower().replace(" ", "")
        if not s:
            continue

        # detect unit
        if s.endswith("kg"):
            val = float(s[:-2])
            plates.append((val, f"{val:g}kg"))
        elif s.endswith("lb") or s.endswith("lbs"):
            if s.endswith("lbs"):
                val = float(s[:-3])
                kg = val * LB_TO_KG
                plates.append((kg, f"{val:g}lb"))
            else:
                val = float(s[:-2])
                kg = val * LB_TO_KG
                plates.append((kg, f"{val:g}lb"))
        else:
            # bare number -> default unit
            val = float(s)
            if default_unit == "kg":
                plates.append((val, f"{val:g}kg"))
            elif default_unit == "lb":
                kg = val * LB_TO_KG
                plates.append((kg, f"{val:g}lb"))
            else:
                raise ValueError("default_unit must be 'kg' or 'lb'")

    return plates

def enumerate_loads_with_combos(plates, tare_kg=0.0, include_zero=False, round_to=3):
    """
    plates: list of (mass_kg, label) items. repeats allowed.
    Returns:
      loads: sorted list of unique totals (kg)
      combo_map: dict total_kg_rounded -> one example list of labels for that total
    """
    n = len(plates)
    combo_map = {}

    start_r = 0 if include_zero else 1

    for r in range(start_r, n + 1):
        for idxs in combinations(range(n), r):
            total = tare_kg + sum(plates[i][0] for i in idxs)
            key = round(total, round_to)

            # Store only the first found combo for each total (you can change this)
            if key not in combo_map:
                labels = [plates[i][1] for i in idxs]
                combo_map[key] = labels

    loads = sorted(combo_map.keys())
    return loads, combo_map

def format_total(total_kg, out_unit="kg", round_to=3):
    if out_unit == "kg":
        return f"{round(total_kg, round_to):g} kg"
    elif out_unit == "lb":
        total_lb = total_kg / LB_TO_KG
        return f"{round(total_lb, round_to):g} lb"
    else:
        raise ValueError("out_unit must be 'kg' or 'lb'")

def main():
    # ---- EDIT THIS ----
    # Option A: write as tokens
    plate_tokens = [
        # examples:
        # "20kg", "20kg", "10kg", "10kg", "5kg", "45lb", "2.5kg"
    ]

    # Option B: quick counts (uncomment and adjust)
    plate_tokens = (
         ["20lb"] * 2 +
         ["35lb"] * 2 +
         ["10kg"]  * 1 +
         ["40lb"] * 2 +
         ["20KG"] * 4
   
     )

    default_unit_for_bare_numbers = "kg"  # if you put "5" with no unit, treat it as kg
    tare = 0.0          # any fixture/platform mass you always include, in kg
    include_zero = False
    rounding = 3        # 3 decimals is usually enough (0.001 kg)
    output_unit = "kg"  # "kg" or "lb"
    show_limit = None   # set to e.g. 50 to print only first 50 and last 50
    # -------------------

    plates = parse_plate_tokens(plate_tokens, default_unit=default_unit_for_bare_numbers)

    loads, combo_map = enumerate_loads_with_combos(
        plates, tare_kg=tare, include_zero=include_zero, round_to=rounding
    )

    if not loads:
        print("No loads found. Add plates to plate_tokens.")
        return

    print(f"Plates count: {len(plates)}")
    print(f"Unique load options: {len(loads)}")
    print(f"Lightest: {format_total(loads[0], output_unit, rounding)}")
    print(f"Heaviest: {format_total(loads[-1], output_unit, rounding)}")
    print("-" * 60)

    def print_row(total_kg):
        combo = combo_map[total_kg]
        total_str = format_total(total_kg, output_unit, rounding)
        print(f"{total_str:>12}  ->  " + " + ".join(combo))

    if show_limit is None or len(loads) <= (show_limit * 2):
        for t in loads:
            print_row(t)
    else:
        for t in loads[:show_limit]:
            print_row(t)
        print("... (middle omitted) ...")
        for t in loads[-show_limit:]:
            print_row(t)

if __name__ == "__main__":
    main()
