#!/usr/bin/env python3
"""
BLE Connection Quality Monitor
Monitors BLE data stream quality, sample rate, and packet loss.
Target: 1000 samples/second with zero data loss.
"""

import asyncio
import struct
import time
import statistics
import sys
from collections import deque
from datetime import datetime
from typing import Dict, List, Optional
from bleak import BleakClient, BleakScanner
import json

# BLE UUIDs
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
DATA_UUID = "87654321-4321-4321-4321-cba987654321"
CMD_UUID = "11111111-2222-3333-4444-555555555555"

# ANSI colors for terminal
class Colors:
    RESET = "\033[0m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    BOLD = "\033[1m"


class BLEQualityMonitor:
    def __init__(self, device_identifier: str, duration_seconds: int = 60):
        self.device_identifier = device_identifier
        self.device_address = None
        self.duration = duration_seconds
        self.start_time = None
        self.running = False
        
        # Packet statistics
        self.packets_received = 0
        self.total_samples = 0
        self.invalid_packets = 0
        self.corrupted_packets = 0
        
        # Timing
        self.last_packet_time = None
        self.packet_intervals: List[float] = []
        
        # Rolling windows for real-time stats (last 1 second)
        self.samples_per_second = deque(maxlen=100)  # Track samples in rolling window
        self.last_second_samples = 0
        self.last_stats_time = None
        
        # Per-second tracking
        self.second_stats: List[Dict] = []
        self.current_second_samples = 0
        self.current_second_start = None
        
        # Packet size tracking
        self.packet_sizes: List[int] = []
        self.samples_per_packet: List[int] = []
        
        # Value tracking (detect stuck/invalid data)
        self.last_values = None
        self.stuck_count = 0
        self.zero_count = 0
        
        # Connection quality
        self.disconnection_count = 0
        self.negotiated_mtu = None
        
        # Quality scoring
        self.quality_issues: List[str] = []
        
    def parse_packet(self, data: bytes) -> Optional[int]:
        """Parse BLE packet and return sample count.
        
        Packet format (161 bytes for 10 samples):
        - byte 0: sample_count (uint8)
        - bytes 1+: samples (16 bytes each)
        """
        if len(data) < 1:
            self.invalid_packets += 1
            return None
        
        sample_count = data[0]
        expected_size = 1 + (sample_count * 16)
        
        if sample_count == 0 or sample_count > 15:
            self.corrupted_packets += 1
            return None
        
        if len(data) < expected_size:
            self.corrupted_packets += 1
            return None
        
        # Track packet info
        self.packet_sizes.append(len(data))
        self.samples_per_packet.append(sample_count)
        
        # Verify data is valid (not all zeros or stuck)
        for i in range(sample_count):
            offset = 1 + i * 16
            values = struct.unpack_from('<8h', data, offset)
            
            # Check for all zeros
            if all(v == 0 for v in values):
                self.zero_count += 1
            
            # Check for stuck values
            if self.last_values is not None:
                if values == self.last_values:
                    self.stuck_count += 1
            self.last_values = values
        
        return sample_count
    
    def data_handler(self, sender, data: bytes):
        """Handle BLE data notifications."""
        now = time.time()
        
        # Calculate packet interval
        if self.last_packet_time is not None:
            interval = (now - self.last_packet_time) * 1000  # ms
            self.packet_intervals.append(interval)
        self.last_packet_time = now
        
        # Parse packet
        sample_count = self.parse_packet(data)
        if sample_count is None:
            return
        
        # Update counters
        self.packets_received += 1
        self.total_samples += sample_count
        self.current_second_samples += sample_count
        
        # Initialize second tracking
        if self.current_second_start is None:
            self.current_second_start = now
            self.last_stats_time = now
        
        # Check if 1 second has passed
        if now - self.current_second_start >= 1.0:
            self.record_second_stats()
            self.current_second_start = now
            self.current_second_samples = sample_count  # Start counting for new second
    
    def record_second_stats(self):
        """Record stats for the completed second."""
        stats = {
            'samples': self.current_second_samples,
            'timestamp': time.time()
        }
        self.second_stats.append(stats)
        
        # Print real-time update
        self.print_realtime_line()
    
    def print_realtime_line(self):
        """Print a single-line real-time status update."""
        if not self.second_stats:
            return
        
        elapsed = time.time() - self.start_time
        last_rate = self.second_stats[-1]['samples']
        
        # Calculate rolling average (last 5 seconds)
        recent = self.second_stats[-5:] if len(self.second_stats) >= 5 else self.second_stats
        avg_rate = sum(s['samples'] for s in recent) / len(recent)
        
        # Color based on rate
        if last_rate >= 990:
            rate_color = Colors.GREEN
            status = "✓"
        elif last_rate >= 900:
            rate_color = Colors.YELLOW
            status = "~"
        else:
            rate_color = Colors.RED
            status = "✗"
        
        # Calculate packet interval stats
        if self.packet_intervals:
            recent_intervals = self.packet_intervals[-100:]
            avg_interval = statistics.mean(recent_intervals)
            jitter = statistics.stdev(recent_intervals) if len(recent_intervals) > 1 else 0
        else:
            avg_interval = 0
            jitter = 0
        
        # Format output
        line = (
            f"\r{status} {elapsed:5.0f}s | "
            f"Rate: {rate_color}{last_rate:4d}{Colors.RESET} sps (avg: {avg_rate:.0f}) | "
            f"Total: {self.total_samples:>8d} | "
            f"Pkts: {self.packets_received:>6d} | "
            f"Interval: {avg_interval:5.1f}ms ±{jitter:4.1f} | "
            f"Err: {self.invalid_packets + self.corrupted_packets}"
        )
        
        print(line, end='', flush=True)
    
    def disconnection_handler(self, client: BleakClient):
        """Handle disconnection events."""
        self.disconnection_count += 1
        print(f"\n{Colors.RED}✗ DISCONNECTED{Colors.RESET} (count: {self.disconnection_count})")
    
    async def find_device(self) -> Optional[str]:
        """Find device by name or return address if already an address."""
        # If it looks like a MAC address, use directly
        if ':' in self.device_identifier and len(self.device_identifier) == 17:
            print(f"Using BLE address: {self.device_identifier}")
            return self.device_identifier
        
        # Scan for device by name
        print(f"Scanning for '{self.device_identifier}'...")
        devices = await BleakScanner.discover(timeout=10.0)
        
        for device in devices:
            if device.name and self.device_identifier.lower() in device.name.lower():
                print(f"✓ Found: {device.name} ({device.address})")
                return device.address
        
        print(f"✗ Device '{self.device_identifier}' not found!")
        print("Available devices:")
        for device in devices:
            name = device.name if device.name else "Unknown"
            print(f"  - {name} ({device.address})")
        return None
    
    def calculate_quality_score(self) -> float:
        """Calculate overall quality score (0-100%)."""
        if not self.second_stats:
            return 0.0
        
        score = 100.0
        issues = []
        
        # 1. Sample rate (target: 1000 sps) - 40% weight
        avg_rate = sum(s['samples'] for s in self.second_stats) / len(self.second_stats)
        rate_score = min(100, (avg_rate / 1000) * 100)
        if rate_score < 95:
            issues.append(f"Low sample rate: {avg_rate:.0f}/1000 sps")
        score_contribution = rate_score * 0.40
        
        # 2. Consistency (low variance) - 20% weight
        rates = [s['samples'] for s in self.second_stats]
        if len(rates) > 1:
            rate_std = statistics.stdev(rates)
            consistency_score = max(0, 100 - rate_std)  # Penalize high variance
            if rate_std > 50:
                issues.append(f"Inconsistent rate: ±{rate_std:.0f} sps")
        else:
            consistency_score = 100
        score_contribution += consistency_score * 0.20
        
        # 3. Packet interval jitter - 20% weight
        if self.packet_intervals:
            jitter = statistics.stdev(self.packet_intervals) if len(self.packet_intervals) > 1 else 0
            jitter_score = max(0, 100 - jitter * 2)  # Penalize high jitter
            if jitter > 10:
                issues.append(f"High jitter: ±{jitter:.1f}ms")
        else:
            jitter_score = 100
        score_contribution += jitter_score * 0.20
        
        # 4. Error rate - 10% weight
        total_packets = self.packets_received + self.invalid_packets + self.corrupted_packets
        if total_packets > 0:
            error_rate = (self.invalid_packets + self.corrupted_packets) / total_packets * 100
            error_score = max(0, 100 - error_rate * 10)  # Heavy penalty for errors
            if error_rate > 1:
                issues.append(f"Packet errors: {error_rate:.1f}%")
        else:
            error_score = 100
        score_contribution += error_score * 0.10
        
        # 5. Connection stability - 10% weight
        if self.disconnection_count > 0:
            stability_score = max(0, 100 - self.disconnection_count * 25)
            issues.append(f"Disconnections: {self.disconnection_count}")
        else:
            stability_score = 100
        score_contribution += stability_score * 0.10
        
        self.quality_issues = issues
        return score_contribution
    
    def generate_report(self) -> Dict:
        """Generate comprehensive quality report."""
        if not self.second_stats:
            return {"error": "No data received"}
        
        elapsed = time.time() - self.start_time
        
        # Sample rate stats
        rates = [s['samples'] for s in self.second_stats]
        avg_rate = statistics.mean(rates)
        min_rate = min(rates)
        max_rate = max(rates)
        rate_std = statistics.stdev(rates) if len(rates) > 1 else 0
        
        # Packet interval stats
        if self.packet_intervals:
            avg_interval = statistics.mean(self.packet_intervals)
            min_interval = min(self.packet_intervals)
            max_interval = max(self.packet_intervals)
            jitter = statistics.stdev(self.packet_intervals) if len(self.packet_intervals) > 1 else 0
        else:
            avg_interval = min_interval = max_interval = jitter = 0
        
        # Packet size stats
        if self.packet_sizes:
            avg_size = statistics.mean(self.packet_sizes)
            unique_sizes = len(set(self.packet_sizes))
        else:
            avg_size = 0
            unique_sizes = 0
        
        # Interval distribution (buckets)
        interval_distribution = {}
        if self.packet_intervals:
            for interval in self.packet_intervals:
                if interval < 5:
                    bucket = "0-5ms"
                elif interval < 10:
                    bucket = "5-10ms"
                elif interval < 15:
                    bucket = "10-15ms"
                elif interval < 20:
                    bucket = "15-20ms"
                elif interval < 30:
                    bucket = "20-30ms"
                elif interval < 50:
                    bucket = "30-50ms"
                elif interval < 100:
                    bucket = "50-100ms"
                else:
                    bucket = "100ms+"
                interval_distribution[bucket] = interval_distribution.get(bucket, 0) + 1
        
        # Samples per packet distribution
        samples_distribution = {}
        if self.samples_per_packet:
            for count in self.samples_per_packet:
                samples_distribution[count] = samples_distribution.get(count, 0) + 1
        
        # Quality score
        quality_score = self.calculate_quality_score()
        
        return {
            'duration_seconds': elapsed,
            'quality_score': quality_score,
            'quality_issues': self.quality_issues,
            'samples': {
                'total': self.total_samples,
                'expected': int(elapsed * 1000),
                'loss_percent': max(0, (1 - self.total_samples / (elapsed * 1000)) * 100) if elapsed > 0 else 0,
                'rate': {
                    'mean': avg_rate,
                    'min': min_rate,
                    'max': max_rate,
                    'std_dev': rate_std
                }
            },
            'packets': {
                'received': self.packets_received,
                'invalid': self.invalid_packets,
                'corrupted': self.corrupted_packets,
                'avg_size_bytes': avg_size,
                'unique_sizes': unique_sizes
            },
            'timing': {
                'interval_ms': {
                    'mean': avg_interval,
                    'min': min_interval,
                    'max': max_interval,
                    'jitter': jitter
                },
                'interval_distribution': interval_distribution
            },
            'packet_samples_distribution': samples_distribution,
            'data_quality': {
                'zero_samples': self.zero_count,
                'stuck_samples': self.stuck_count
            },
            'connection': {
                'disconnections': self.disconnection_count,
                'mtu': self.negotiated_mtu
            },
            'per_second_rates': rates
        }
    
    def print_report(self, report: Dict):
        """Print formatted report."""
        print("\n")
        print("=" * 70)
        print(f"{Colors.BOLD}BLE DATA STREAM QUALITY REPORT{Colors.RESET}")
        print("=" * 70)
        
        if 'error' in report:
            print(f"\n{Colors.RED}Error: {report['error']}{Colors.RESET}")
            return
        
        # Quality Score (big display)
        score = report['quality_score']
        if score >= 95:
            score_color = Colors.GREEN
            grade = "EXCELLENT"
        elif score >= 85:
            score_color = Colors.GREEN
            grade = "GOOD"
        elif score >= 70:
            score_color = Colors.YELLOW
            grade = "FAIR"
        else:
            score_color = Colors.RED
            grade = "POOR"
        
        print(f"\n{Colors.BOLD}QUALITY SCORE: {score_color}{score:.1f}% ({grade}){Colors.RESET}")
        
        if report['quality_issues']:
            print(f"\n{Colors.YELLOW}Issues:{Colors.RESET}")
            for issue in report['quality_issues']:
                print(f"  • {issue}")
        
        # Sample Statistics
        print(f"\n{Colors.CYAN}─── SAMPLE RATE ───{Colors.RESET}")
        samples = report['samples']
        print(f"  Target:     1000 sps")
        print(f"  Achieved:   {samples['rate']['mean']:.1f} sps (min: {samples['rate']['min']}, max: {samples['rate']['max']})")
        print(f"  Std Dev:    ±{samples['rate']['std_dev']:.1f} sps")
        print(f"  Total:      {samples['total']:,} samples in {report['duration_seconds']:.1f}s")
        
        loss = samples['loss_percent']
        loss_color = Colors.GREEN if loss < 1 else (Colors.YELLOW if loss < 5 else Colors.RED)
        print(f"  Data Loss:  {loss_color}{loss:.2f}%{Colors.RESET}")
        
        # Timing
        print(f"\n{Colors.CYAN}─── TIMING ───{Colors.RESET}")
        timing = report['timing']['interval_ms']
        print(f"  Packet Interval: {timing['mean']:.1f}ms (expected: ~10ms)")
        print(f"  Range:           {timing['min']:.1f}ms - {timing['max']:.1f}ms")
        jitter_color = Colors.GREEN if timing['jitter'] < 5 else (Colors.YELLOW if timing['jitter'] < 15 else Colors.RED)
        print(f"  Jitter:          {jitter_color}±{timing['jitter']:.1f}ms{Colors.RESET}")
        
        # Interval Distribution
        if 'interval_distribution' in report['timing'] and report['timing']['interval_distribution']:
            print(f"\n{Colors.CYAN}─── INTERVAL DISTRIBUTION ───{Colors.RESET}")
            interval_dist = report['timing']['interval_distribution']
            total_intervals = sum(interval_dist.values())
            # Sort by interval range
            order = ["0-5ms", "5-10ms", "10-15ms", "15-20ms", "20-30ms", "30-50ms", "50-100ms", "100ms+"]
            for bucket in order:
                if bucket in interval_dist:
                    count = interval_dist[bucket]
                    pct = (count / total_intervals) * 100
                    bar_len = int(pct / 2)
                    bar = "█" * bar_len
                    # Color based on bucket (green for 5-15ms, yellow for others, red for 50ms+)
                    if bucket in ["5-10ms", "10-15ms"]:
                        color = Colors.GREEN
                    elif bucket in ["0-5ms", "15-20ms", "20-30ms"]:
                        color = Colors.YELLOW
                    else:
                        color = Colors.RED
                    print(f"  {bucket:>8s}: {color}{bar} {count:>5d} ({pct:5.1f}%){Colors.RESET}")
        
        # Packets
        print(f"\n{Colors.CYAN}─── PACKETS ───{Colors.RESET}")
        packets = report['packets']
        print(f"  Received:   {packets['received']:,}")
        print(f"  Avg Size:   {packets['avg_size_bytes']:.0f} bytes")
        errors = packets['invalid'] + packets['corrupted']
        error_color = Colors.GREEN if errors == 0 else Colors.RED
        print(f"  Errors:     {error_color}{errors}{Colors.RESET} (invalid: {packets['invalid']}, corrupted: {packets['corrupted']})")
        
        # Samples per Packet Distribution
        if 'packet_samples_distribution' in report and report['packet_samples_distribution']:
            print(f"\n{Colors.CYAN}─── SAMPLES PER PACKET ───{Colors.RESET}")
            samples_dist = report['packet_samples_distribution']
            total_pkts = sum(samples_dist.values())
            for count in sorted(samples_dist.keys()):
                num = samples_dist[count]
                pct = (num / total_pkts) * 100
                bar_len = int(pct / 2)
                bar = "█" * bar_len
                # Color: green for 10, yellow for 8-9, red for others
                if count == 10:
                    color = Colors.GREEN
                elif count >= 8:
                    color = Colors.YELLOW
                else:
                    color = Colors.RED
                print(f"  {count:2d} samples: {color}{bar} {num:>5d} ({pct:5.1f}%){Colors.RESET}")
        
        # Data Quality
        print(f"\n{Colors.CYAN}─── DATA QUALITY ───{Colors.RESET}")
        dq = report['data_quality']
        print(f"  Zero samples:  {dq['zero_samples']}")
        print(f"  Stuck samples: {dq['stuck_samples']}")
        
        # Connection
        print(f"\n{Colors.CYAN}─── CONNECTION ───{Colors.RESET}")
        conn = report['connection']
        disc_color = Colors.GREEN if conn['disconnections'] == 0 else Colors.RED
        print(f"  Disconnections: {disc_color}{conn['disconnections']}{Colors.RESET}")
        if conn['mtu']:
            print(f"  MTU: {conn['mtu']} bytes")
        
        # Per-second rate histogram
        if len(report['per_second_rates']) > 1:
            print(f"\n{Colors.CYAN}─── RATE HISTOGRAM (last 20 seconds) ───{Colors.RESET}")
            rates = report['per_second_rates'][-20:]
            max_rate = max(rates) if rates else 1000
            for i, rate in enumerate(rates):
                bar_len = int((rate / max(max_rate, 1000)) * 40)
                bar = "█" * bar_len
                if rate >= 990:
                    color = Colors.GREEN
                elif rate >= 900:
                    color = Colors.YELLOW
                else:
                    color = Colors.RED
                print(f"  {i+1:2d}s: {color}{bar} {rate}{Colors.RESET}")
        
        print("\n" + "=" * 70)
    
    async def run_monitoring(self):
        """Run the monitoring session."""
        print(f"\n{Colors.BOLD}BLE Data Stream Quality Monitor{Colors.RESET}")
        print(f"Target: 1000 samples/second with zero loss")
        print(f"Duration: {self.duration} seconds")
        print("-" * 50)
        
        # Find device
        device_address = await self.find_device()
        if device_address is None:
            return
        
        self.device_address = device_address
        
        try:
            async with BleakClient(
                device_address,
                disconnected_callback=self.disconnection_handler
            ) as client:
                self.start_time = time.time()
                self.running = True
                
                # Get MTU
                if hasattr(client, 'mtu_size'):
                    self.negotiated_mtu = client.mtu_size
                    print(f"MTU: {self.negotiated_mtu} bytes")
                
                # Subscribe to data
                await client.start_notify(DATA_UUID, self.data_handler)
                print(f"✓ Subscribed to data stream")
                
                # Send START command
                await client.write_gatt_char(CMD_UUID, b"ALL_START")
                print(f"✓ Sent ALL_START command")
                print("-" * 50)
                print("Monitoring... (Ctrl+C to stop early)")
                print()
                
                # Header for real-time display
                print(f"  Time  |  Rate (sps)  |   Total   | Packets | Interval     | Err")
                print("-" * 70)
                
                # Monitor for specified duration
                await asyncio.sleep(self.duration)
                
                # Stop
                print()  # New line after real-time display
                await client.write_gatt_char(CMD_UUID, b"ALL_STOP")
                print(f"\n✓ Sent ALL_STOP command")
                
                await asyncio.sleep(0.5)
                
        except KeyboardInterrupt:
            print(f"\n\n{Colors.YELLOW}Stopped by user{Colors.RESET}")
        except Exception as e:
            print(f"\n{Colors.RED}Error: {e}{Colors.RESET}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
            
            # Record final second if needed
            if self.current_second_samples > 0 and self.current_second_start:
                elapsed_in_second = time.time() - self.current_second_start
                if elapsed_in_second > 0.5:  # Only record if > 0.5 second
                    self.record_second_stats()
            
            # Generate and print report
            report = self.generate_report()
            self.print_report(report)
            
            # Save report
            if 'error' not in report:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = f"ble_quality_report_{timestamp}.json"
                with open(filename, 'w') as f:
                    json.dump(report, f, indent=2)
                print(f"\n📄 Report saved to: {filename}")


async def main():
    if len(sys.argv) < 2:
        print("BLE Data Stream Quality Monitor")
        print("-" * 40)
        print("\nUsage: python ble_connection_quality_monitor.py <DEVICE> [DURATION]")
        print("\nArguments:")
        print("  DEVICE    BLE device address (XX:XX:XX:XX:XX:XX) or name")
        print("  DURATION  Test duration in seconds (default: 60)")
        print("\nExamples:")
        print("  python ble_connection_quality_monitor.py LoadCell_BLE_Server 30")
        print("  python ble_connection_quality_monitor.py AA:BB:CC:DD:EE:FF 60")
        print("  python ble_connection_quality_monitor.py LoadCell 120")
        sys.exit(1)
    
    device = sys.argv[1]
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 60
    
    monitor = BLEQualityMonitor(device, duration)
    await monitor.run_monitoring()


if __name__ == "__main__":
    asyncio.run(main())
