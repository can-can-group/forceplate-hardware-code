# ESP32 BLE Load Cell Server - Client Integration Guide

This document describes how to connect to the ESP32 BLE Load Cell Server, subscribe to sensor data, send commands, and receive JSON responses.

---

## BLE Connection Details

### Device Information

| Property | Value |
|----------|-------|
| **Device Name** | `LoadCell_BLE_Server` |
| **Service UUID** | `12345678-1234-1234-1234-123456789abc` |

### Characteristics

| Characteristic | UUID | Properties | Purpose |
|----------------|------|------------|---------|
| **Data** | `87654321-4321-4321-4321-cba987654321` | NOTIFY | Binary sensor data stream |
| **Cmd** | `11111111-2222-3333-4444-555555555555` | WRITE, NOTIFY | Send commands, receive JSON responses |

---

## Quick Start

### 1. Connect
Scan for `LoadCell_BLE_Server` and connect.

### 2. Subscribe to Cmd Characteristic
Enable notifications on `11111111-2222-3333-4444-555555555555` to receive command responses.

### 3. Send Commands
Write text to the same Cmd characteristic (e.g., `LOCAL_PING`).

### 4. Receive JSON Response
Response arrives as notification: `{"target":"LOCAL","cmd":"PING","ok":true,"ms":45}`

### 5. Subscribe to Data (Optional)
Enable notifications on `87654321-4321-4321-4321-cba987654321` for sensor data stream.

---

## JSON Response Format

All command responses are JSON objects with these fields:

| Field | Type | Description |
|-------|------|-------------|
| `target` | string | `"LOCAL"`, `"REMOTE"`, `"ALL"`, or `"BLE"` |
| `cmd` | string | Command name (e.g., `"PING"`, `"TARE"`, `"SHOW"`) |
| `ok` | boolean | Success status |
| `ms` | number | Round-trip time in milliseconds |
| `err` | string | Error message (only if `ok` is false) |

### Response Examples

```json
// PING success
{"target":"LOCAL","cmd":"PING","ok":true,"ms":45}

// TARE success
{"target":"LOCAL","cmd":"TARE","ok":true,"ms":650}

// TARE failure (timeout)
{"target":"LOCAL","cmd":"TARE","ok":false,"err":"TIMEOUT","ms":5001}

// SHOW - calibration status
{"target":"LOCAL","cmd":"SHOW","lc":[
  {"off":38853,"a":0.015743,"n":2},
  {"off":12345,"a":0.014892,"n":2},
  {"off":-1234,"a":0.016021,"n":2},
  {"off":-5678,"a":0.015156,"n":2}
],"ms":52}

// READ - calibrated values (10g units)
{"target":"LOCAL","cmd":"READ","v":[100,150,120,130,500],"ms":35}

// PTS - point counts per channel
{"target":"LOCAL","cmd":"PTS","n":[2,2,2,2],"ms":30}

// ADD calibration point
{"target":"LOCAL","cmd":"ADD","ok":true,"ms":850}

// Control command
{"target":"LOCAL","cmd":"START","ok":true,"ms":120}  // Response to LOCAL_START

// LED control commands
{"target":"LOCAL","cmd":"LED_ON","ok":true,"ms":45}
{"target":"ALL","cmd":"LED_OFF","ok":true,"ms":48}

// Mock data commands
{"target":"LOCAL","cmd":"MOCK_ON","ok":true,"ms":42}
{"target":"ALL","cmd":"MOCK_OFF","ok":true,"ms":45}

// STATUS - comprehensive system status
{"target":"BLE","cmd":"STATUS","ok":true,"local":{"state":"RUNNING","led":"ON","bat":{"v":4.12,"pct":85.0}},"remote":{"state":"RUNNING","led":"ON","bat":{"v":3.98,"pct":72.0}},"ble":"CONNECTED","uptime":3600,"ms":245}

// BAT - battery only
{"target":"BLE","cmd":"BAT","ok":true,"local":{"v":4.12,"pct":85.0},"remote":{"v":3.98,"pct":72.0},"ms":0}
```

---

## Sensor Data Format (Binary)

The Data characteristic sends batched load cell samples at ~100 packets/second (1000 samples/second).

### Packet Structure

```
Byte 0:        sample_count (1-10)
Bytes 1-160:   samples[sample_count]
```

**Maximum packet size: 161 bytes**

### Each Sample (16 bytes)

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| 0-1 | 2 bytes | int16 LE | Local Load Cell 1 |
| 2-3 | 2 bytes | int16 LE | Local Load Cell 2 |
| 4-5 | 2 bytes | int16 LE | Local Load Cell 3 |
| 6-7 | 2 bytes | int16 LE | Local Load Cell 4 |
| 8-9 | 2 bytes | int16 LE | Remote Load Cell 5 |
| 10-11 | 2 bytes | int16 LE | Remote Load Cell 6 |
| 12-13 | 2 bytes | int16 LE | Remote Load Cell 7 |
| 14-15 | 2 bytes | int16 LE | Remote Load Cell 8 |

**Value Range:** -32768 to +32767 (int16)

---

## Available Commands

### Control Commands

| Command | Description |
|---------|-------------|
| `LOCAL_START` | Start local data acquisition |
| `LOCAL_STOP` | Stop local data acquisition |
| `LOCAL_RESTART` | Restart local Teensy |
| `LOCAL_RESET` | Reset local Teensy |
| `REMOTE_START` | Start remote data acquisition |
| `REMOTE_STOP` | Stop remote data acquisition |
| `REMOTE_RESTART` | Restart remote Teensy |
| `REMOTE_RESET` | Reset remote Teensy |
| `ALL_START` | Start both Teensys |
| `ALL_STOP` | Stop both Teensys |
| `ALL_RESTART` | Restart both Teensys |
| `ALL_RESET` | Reset both Teensys |

### LED Control Commands

| Command | Description |
|---------|-------------|
| `ALL_LED_ON` | Turn on LEDs on both Teensy devices |
| `ALL_LED_OFF` | Turn off LEDs on both Teensy devices |

**Note:** LED commands control the WS2812 RGB LEDs on each Teensy device. When LEDs are enabled, they display status information (idle, running, calibration, etc.). When disabled, all LEDs are turned off. LEDs do not animate during data acquisition to maintain precise 1000 Hz sampling.

### Mock Data Commands

| Command | Description |
|---------|-------------|
| `ALL_MOCK_ON` | Enable mock data generation on both Teensy devices |
| `ALL_MOCK_OFF` | Disable mock data generation on both Teensy devices |

**Note:** Mock data generation replaces physical load cell readings with synthetic waveforms for testing data rates and connection reliability without requiring physical hardware. When enabled, LEDs display inverted colors (yellow/orange instead of pink/blue) to indicate mock data mode.

### Ping Commands

| Command | Response |
|---------|----------|
| `LOCAL_PING` | `{"target":"LOCAL","cmd":"PING","ok":true,"ms":XX}` |
| `REMOTE_PING` | `{"target":"REMOTE","cmd":"PING","ok":true,"ms":XX}` |

### Calibration Commands

| Command | Description |
|---------|-------------|
| `LOCAL_CAL_TARE` | Zero load cells (no weight) |
| `LOCAL_CAL_SHOW` | Show calibration status |
| `LOCAL_CAL_READ` | Read calibrated values (10g units) |
| `LOCAL_CAL_ADD_<kg>` | Add calibration point (e.g., `LOCAL_CAL_ADD_10`) |
| `LOCAL_CAL_ADD_CH_<ch>_<kg>` | Add point to specific channel |
| `LOCAL_CAL_CLEAR` | Clear all calibration |
| `LOCAL_CAL_POINTS` | Show point counts |
| `REMOTE_CAL_*` | Same commands for remote Teensy |

### Status Command

| Command | Description |
|---------|-------------|
| `STATUS` | Get comprehensive system status (state, LEDs, battery, uptime) |

**STATUS Response Format:**
```json
{
  "target": "BLE",
  "cmd": "STATUS",
  "ok": true,
  "local": {
    "state": "RUNNING",
    "led": "ON",
    "bat": {"v": 4.12, "pct": 85.0}
  },
  "remote": {
    "state": "RUNNING",
    "led": "ON", 
    "bat": {"v": 3.98, "pct": 72.0}
  },
  "ble": "CONNECTED",
  "uptime": 3600,
  "ms": 245
}
```

| Field | Description |
|-------|-------------|
| `local.state` | Local Teensy acquisition state (`RUNNING`, `STOPPED`, `UNKNOWN`) |
| `local.led` | Local LED status (`ON`, `OFF`, `UNKNOWN`) |
| `local.bat.v` | Local battery voltage (V) |
| `local.bat.pct` | Local battery percentage (0-100%) |
| `remote.state` | Remote Teensy acquisition state |
| `remote.led` | Remote LED status |
| `remote.bat.v` | Remote battery voltage (V) |
| `remote.bat.pct` | Remote battery percentage (0-100%) |
| `ble` | BLE connection status (`CONNECTED`, `DISCONNECTED`) |
| `uptime` | System uptime in seconds |
| `ms` | Round-trip time for status queries (milliseconds) |

### Other Commands

| Command | Description |
|---------|-------------|
| `BAT` | Get battery status only (subset of STATUS) |
| `HELP` | Show available commands (printed to Serial) |

---

## Load Cell Data Stream - Complete Guide

This section provides a complete guide on how to receive, parse, and format the load cell data stream into human-readable time-series data.

### Step 1: Subscribe to Data Characteristic

First, enable notifications on the Data characteristic to receive the binary data stream:

```python
# Python example
await client.start_notify(DATA_UUID, data_handler)
```

### Step 2: Parse Binary Packet

Each packet contains 1-10 samples. Parse the packet structure:

```python
def parse_sensor_data(data: bytes) -> list:
    """
    Parse binary sensor data packet.
    
    Packet format:
    - Byte 0: sample_count (1-10)
    - Bytes 1-160: samples (16 bytes each)
    
    Each sample: 8 int16 values (little-endian)
    - Local: LC1, LC2, LC3, LC4 (bytes 0-7)
    - Remote: LC5, LC6, LC7, LC8 (bytes 8-15)
    """
    if len(data) < 1:
        return []
    
    sample_count = data[0]
    samples = []
    
    for i in range(sample_count):
        offset = 1 + (i * 16)
        if offset + 16 > len(data):
            break
            
        # Unpack 8 int16 values (little-endian)
        values = struct.unpack_from('<8h', data, offset)
        
        samples.append({
            'local': {
                'lc1': int(values[0]),
                'lc2': int(values[1]),
                'lc3': int(values[2]),
                'lc4': int(values[3])
            },
            'remote': {
                'lc5': int(values[4]),
                'lc6': int(values[5]),
                'lc7': int(values[6]),
                'lc8': int(values[7])
            }
        })
    
    return samples
```

### Step 3: Format as Human-Readable Time Series

Convert parsed data into readable format with timestamps:

```python
import time
from datetime import datetime

class LoadCellDataLogger:
    def __init__(self):
        self.start_time = time.time()
        self.sample_count = 0
        
    def format_sample(self, sample: dict, timestamp: float = None) -> str:
        """Format a single sample as human-readable string."""
        if timestamp is None:
            timestamp = time.time()
        
        elapsed = timestamp - self.start_time
        self.sample_count += 1
        
        # Format as CSV-like string
        local = sample['local']
        remote = sample['remote']
        
        return (
            f"{self.sample_count:6d}, "
            f"{elapsed:8.3f}, "
            f"L:{local['lc1']:6d} {local['lc2']:6d} {local['lc3']:6d} {local['lc4']:6d}, "
            f"R:{remote['lc5']:6d} {remote['lc6']:6d} {remote['lc7']:6d} {remote['lc8']:6d}"
        )
    
    def format_sample_json(self, sample: dict, timestamp: float = None) -> dict:
        """Format a single sample as JSON object."""
        if timestamp is None:
            timestamp = time.time()
        
        elapsed = timestamp - self.start_time
        
        return {
            'sample': self.sample_count,
            'timestamp': timestamp,
            'elapsed_ms': elapsed * 1000,
            'local': {
                'lc1': sample['local']['lc1'],
                'lc2': sample['local']['lc2'],
                'lc3': sample['local']['lc3'],
                'lc4': sample['local']['lc4']
            },
            'remote': {
                'lc5': sample['remote']['lc5'],
                'lc6': sample['remote']['lc6'],
                'lc7': sample['remote']['lc7'],
                'lc8': sample['remote']['lc8']
            }
        }
```

### Step 4: Complete Data Handler

Complete example with data logging:

```python
import struct
import json
import csv
from datetime import datetime
from bleak import BleakClient

SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
DATA_UUID = "87654321-4321-4321-4321-cba987654321"
CMD_UUID = "11111111-2222-3333-4444-555555555555"

class LoadCellDataStream:
    def __init__(self, output_file=None, format='csv'):
        self.output_file = output_file
        self.format = format
        self.start_time = time.time()
        self.sample_count = 0
        self.csv_writer = None
        
        if output_file and format == 'csv':
            self.csv_file = open(output_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write header
            self.csv_writer.writerow([
                'Sample', 'Elapsed_ms', 'Timestamp',
                'Local_LC1', 'Local_LC2', 'Local_LC3', 'Local_LC4',
                'Remote_LC5', 'Remote_LC6', 'Remote_LC7', 'Remote_LC8'
            ])
    
    def parse_packet(self, data: bytes) -> list:
        """Parse binary packet into list of samples."""
        if len(data) < 1:
            return []
        
        sample_count = data[0]
        samples = []
        
        for i in range(sample_count):
            offset = 1 + (i * 16)
            if offset + 16 > len(data):
                break
                
            values = struct.unpack_from('<8h', data, offset)
            
            samples.append({
                'local': [int(v) for v in values[0:4]],
                'remote': [int(v) for v in values[4:8]]
            })
        
        return samples
    
    def process_samples(self, samples: list):
        """Process and log samples."""
        timestamp = time.time()
        
        for sample in samples:
            self.sample_count += 1
            elapsed_ms = (timestamp - self.start_time) * 1000
            
            # Print to console (human-readable)
            local = sample['local']
            remote = sample['remote']
            print(
                f"Sample {self.sample_count:6d} | "
                f"Elapsed: {elapsed_ms:8.1f}ms | "
                f"Local: [{local[0]:6d} {local[1]:6d} {local[2]:6d} {local[3]:6d}] | "
                f"Remote: [{remote[0]:6d} {remote[1]:6d} {remote[2]:6d} {remote[3]:6d}]"
            )
            
            # Write to CSV file if enabled
            if self.csv_writer:
                self.csv_writer.writerow([
                    self.sample_count,
                    f"{elapsed_ms:.1f}",
                    datetime.fromtimestamp(timestamp).isoformat(),
                    local[0], local[1], local[2], local[3],
                    remote[0], remote[1], remote[2], remote[3]
                ])
                self.csv_file.flush()
    
    def data_handler(self, sender, data: bytes):
        """BLE notification handler for data characteristic."""
        samples = self.parse_packet(data)
        if samples:
            self.process_samples(samples)
    
    def close(self):
        """Close output file if open."""
        if self.csv_file:
            self.csv_file.close()

# Usage example
async def main():
    stream = LoadCellDataStream(output_file='loadcell_data.csv', format='csv')
    
    async with BleakClient("XX:XX:XX:XX:XX:XX") as client:
        # Subscribe to data stream
        await client.start_notify(DATA_UUID, stream.data_handler)
        
        # Start data acquisition
        await client.write_gatt_char(CMD_UUID, b"ALL_START")
        
        # Run for 60 seconds
        await asyncio.sleep(60)
        
        # Stop data acquisition
        await client.write_gatt_char(CMD_UUID, b"ALL_STOP")
    
    stream.close()
    print(f"\nTotal samples collected: {stream.sample_count}")
```

### Step 5: Output Formats

#### CSV Format
```
Sample, Elapsed_ms, Timestamp, Local_LC1, Local_LC2, Local_LC3, Local_LC4, Remote_LC5, Remote_LC6, Remote_LC7, Remote_LC8
1, 0.0, 2025-12-19T10:30:00.123, 1234, 5678, -1234, 8901, 2345, 6789, -2345, 9012
2, 1.0, 2025-12-19T10:30:00.124, 1235, 5679, -1233, 8902, 2346, 6790, -2344, 9013
```

#### JSON Format
```json
{
  "sample": 1,
  "timestamp": 1734604200.123,
  "elapsed_ms": 0.0,
  "local": {"lc1": 1234, "lc2": 5678, "lc3": -1234, "lc4": 8901},
  "remote": {"lc5": 2345, "lc6": 6789, "lc7": -2345, "lc8": 9012}
}
```

#### Console Output (Human-Readable)
```
Sample      1 | Elapsed:      0.0ms | Local: [  1234   5678  -1234   8901] | Remote: [  2345   6789  -2345   9012]
Sample      2 | Elapsed:      1.0ms | Local: [  1235   5679  -1233   8902] | Remote: [  2346   6790  -2344   9013]
```

### JavaScript Example

```javascript
const DATA_UUID = '87654321-4321-4321-4321-cba987654321';

class LoadCellParser {
    constructor() {
        this.startTime = Date.now();
        this.sampleCount = 0;
    }
    
    parsePacket(dataView) {
        const sampleCount = dataView.getUint8(0);
        const samples = [];
        
        for (let i = 0; i < sampleCount; i++) {
            const offset = 1 + (i * 16);
            if (offset + 16 > dataView.byteLength) break;
            
            const local = [
                dataView.getInt16(offset + 0, true),  // LC1
                dataView.getInt16(offset + 2, true),  // LC2
                dataView.getInt16(offset + 4, true),  // LC3
                dataView.getInt16(offset + 6, true)   // LC4
            ];
            
            const remote = [
                dataView.getInt16(offset + 8, true),  // LC5
                dataView.getInt16(offset + 10, true), // LC6
                dataView.getInt16(offset + 12, true), // LC7
                dataView.getInt16(offset + 14, true)  // LC8
            ];
            
            samples.push({ local, remote });
        }
        
        return samples;
    }
    
    formatSample(sample) {
        this.sampleCount++;
        const elapsed = Date.now() - this.startTime;
        
        const localStr = sample.local.map(v => v.toString().padStart(6)).join(' ');
        const remoteStr = sample.remote.map(v => v.toString().padStart(6)).join(' ');
        
        return `Sample ${this.sampleCount.toString().padStart(6)} | ` +
               `Elapsed: ${elapsed.toString().padStart(8)}ms | ` +
               `Local: [${localStr}] | Remote: [${remoteStr}]`;
    }
    
    handleData(event) {
        const dataView = new DataView(event.target.value.buffer);
        const samples = this.parsePacket(dataView);
        
        samples.forEach(sample => {
            console.log(this.formatSample(sample));
        });
    }
}

// Usage
const parser = new LoadCellParser();
await dataChar.startNotifications();
dataChar.addEventListener('characteristicvaluechanged', (event) => {
    parser.handleData(event);
});
```

### Data Rate Information

- **Sample Rate:** 1000 Hz per channel (4000 Hz total for 4 channels)
- **Packet Rate:** ~100 packets/second
- **Samples per Packet:** 1-10 samples
- **Packet Size:** 1 + (sample_count × 16) bytes (max 161 bytes)
- **Data Throughput:** ~16 KB/s

### Tips for Data Collection

1. **Buffer Management:** Process samples quickly to avoid buffer overflow
2. **File I/O:** Use buffered writes or async I/O for CSV/JSON logging
3. **Real-time Display:** Update UI in batches (e.g., every 10-20 samples)
4. **Data Validation:** Check sample_count and packet size before parsing
5. **Error Handling:** Handle incomplete packets gracefully

---

## Parsing Examples

### Python (with bleak)

```python
import struct
import json
from bleak import BleakClient

SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
DATA_UUID = "87654321-4321-4321-4321-cba987654321"
CMD_UUID = "11111111-2222-3333-4444-555555555555"

def parse_sensor_data(data: bytes) -> list:
    """Parse binary sensor data packet."""
    sample_count = data[0]
    samples = []
    for i in range(sample_count):
        offset = 1 + (i * 16)
        values = struct.unpack_from('<8h', data, offset)
        samples.append({
            'local': list(values[0:4]),
            'remote': list(values[4:8])
        })
    return samples

def parse_json_response(data: bytes) -> dict:
    """Parse JSON command response."""
    return json.loads(data.decode('utf-8'))

async def main():
    async with BleakClient("XX:XX:XX:XX:XX:XX") as client:
        # Handler for command responses (JSON)
        def cmd_handler(sender, data):
            response = parse_json_response(data)
            print(f"Response: {response}")
        
        # Handler for sensor data (binary)
        def data_handler(sender, data):
            samples = parse_sensor_data(data)
            for s in samples:
                print(f"L:{s['local']} R:{s['remote']}")
        
        # Subscribe to responses
        await client.start_notify(CMD_UUID, cmd_handler)
        
        # Send command
        await client.write_gatt_char(CMD_UUID, b"LOCAL_PING")
        
        # Control LEDs
        await client.write_gatt_char(CMD_UUID, b"ALL_LED_ON")
        
        # Enable mock data for testing (optional)
        # await client.write_gatt_char(CMD_UUID, b"ALL_MOCK_ON")
        
        # Subscribe to sensor data
        await client.start_notify(DATA_UUID, data_handler)
        await client.write_gatt_char(CMD_UUID, b"ALL_START")
        
        # Keep running...
        await asyncio.sleep(60)
```

### JavaScript (Web Bluetooth)

```javascript
const SERVICE_UUID = '12345678-1234-1234-1234-123456789abc';
const DATA_UUID = '87654321-4321-4321-4321-cba987654321';
const CMD_UUID = '11111111-2222-3333-4444-555555555555';

function parseSensorData(dataView) {
    const sampleCount = dataView.getUint8(0);
    const samples = [];
    for (let i = 0; i < sampleCount; i++) {
        const offset = 1 + (i * 16);
        samples.push({
            local: [
                dataView.getInt16(offset + 0, true),
                dataView.getInt16(offset + 2, true),
                dataView.getInt16(offset + 4, true),
                dataView.getInt16(offset + 6, true),
            ],
            remote: [
                dataView.getInt16(offset + 8, true),
                dataView.getInt16(offset + 10, true),
                dataView.getInt16(offset + 12, true),
                dataView.getInt16(offset + 14, true),
            ]
        });
    }
    return samples;
}

async function connect() {
    const device = await navigator.bluetooth.requestDevice({
        filters: [{ name: 'LoadCell_BLE_Server' }],
        optionalServices: [SERVICE_UUID]
    });
    
    const server = await device.gatt.connect();
    const service = await server.getPrimaryService(SERVICE_UUID);
    
    // Get characteristics
    const cmdChar = await service.getCharacteristic(CMD_UUID);
    const dataChar = await service.getCharacteristic(DATA_UUID);
    
    // Subscribe to command responses (JSON)
    await cmdChar.startNotifications();
    cmdChar.addEventListener('characteristicvaluechanged', (event) => {
        const decoder = new TextDecoder();
        const json = JSON.parse(decoder.decode(event.target.value));
        console.log('Response:', json);
    });
    
    // Subscribe to sensor data (binary)
    await dataChar.startNotifications();
    dataChar.addEventListener('characteristicvaluechanged', (event) => {
        const samples = parseSensorData(event.target.value);
        console.log('Samples:', samples);
    });
    
    // Send command
    const encoder = new TextEncoder();
    await cmdChar.writeValue(encoder.encode('LOCAL_PING'));
    // Control LEDs
    await cmdChar.writeValue(encoder.encode('ALL_LED_ON'));
    
    // Enable mock data for testing (optional)
    // await cmdChar.writeValue(encoder.encode('ALL_MOCK_ON'));
    
    // Start data acquisition
    await cmdChar.writeValue(encoder.encode('ALL_START'));
}
```

### Flutter (Dart)

```dart
import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

const serviceUuid = Guid('12345678-1234-1234-1234-123456789abc');
const dataUuid = Guid('87654321-4321-4321-4321-cba987654321');
const cmdUuid = Guid('11111111-2222-3333-4444-555555555555');

List<Map<String, List<int>>> parseSensorData(Uint8List data) {
  final sampleCount = data[0];
  final samples = <Map<String, List<int>>>[];
  
  for (var i = 0; i < sampleCount; i++) {
    final offset = 1 + (i * 16);
    final byteData = ByteData.sublistView(data, offset, offset + 16);
    
    samples.add({
      'local': [
        byteData.getInt16(0, Endian.little),
        byteData.getInt16(2, Endian.little),
        byteData.getInt16(4, Endian.little),
        byteData.getInt16(6, Endian.little),
      ],
      'remote': [
        byteData.getInt16(8, Endian.little),
        byteData.getInt16(10, Endian.little),
        byteData.getInt16(12, Endian.little),
        byteData.getInt16(14, Endian.little),
      ],
    });
  }
  return samples;
}

Map<String, dynamic> parseJsonResponse(Uint8List data) {
  return jsonDecode(utf8.decode(data));
}

Future<void> connectAndStream(BluetoothDevice device) async {
  await device.connect();
  final services = await device.discoverServices();
  final service = services.firstWhere((s) => s.uuid == serviceUuid);
  
  final cmdChar = service.characteristics.firstWhere((c) => c.uuid == cmdUuid);
  final dataChar = service.characteristics.firstWhere((c) => c.uuid == dataUuid);
  
  // Subscribe to command responses
  await cmdChar.setNotifyValue(true);
  cmdChar.value.listen((data) {
    final response = parseJsonResponse(Uint8List.fromList(data));
    print('Response: $response');
  });
  
  // Subscribe to sensor data
  await dataChar.setNotifyValue(true);
  dataChar.value.listen((data) {
    final samples = parseSensorData(Uint8List.fromList(data));
    print('Samples: $samples');
  });
  
  // Send commands
  await cmdChar.write(utf8.encode('LOCAL_PING'));
  // Control LEDs
  await cmdChar.write(utf8.encode('ALL_LED_ON'));
  
  // Enable mock data for testing (optional)
  // await cmdChar.write(utf8.encode('ALL_MOCK_ON'));
  
  // Start data acquisition
  await cmdChar.write(utf8.encode('ALL_START'));
}
```

---

## Calibration Workflow

### Step 1: Clear Previous Calibration
```
Send: LOCAL_CAL_CLEAR
Response: {"target":"LOCAL","cmd":"CLEAR","ok":true,"ms":XX}
```

### Step 2: Tare (Zero with No Load)
Remove all weight from the platform.
```
Send: LOCAL_CAL_TARE
Response: {"target":"LOCAL","cmd":"TARE","ok":true,"ms":650}
```

### Step 3: Add Known Weights
Place known weight (e.g., 10 kg) on platform.
```
Send: LOCAL_CAL_ADD_10
Response: {"target":"LOCAL","cmd":"ADD","ok":true,"ms":850}
```

Repeat with more weights for better accuracy:
```
Send: LOCAL_CAL_ADD_20
Send: LOCAL_CAL_ADD_50
```

### Step 4: Verify Calibration
```
Send: LOCAL_CAL_SHOW
Response: {"target":"LOCAL","cmd":"SHOW","lc":[
  {"off":38853,"a":0.015743,"n":3},
  ...
],"ms":52}
```

The `n` field shows number of calibration points per channel.

### Step 5: Read Calibrated Values
```
Send: LOCAL_CAL_READ
Response: {"target":"LOCAL","cmd":"READ","v":[100,150,120,130,500],"ms":35}
```

Values are in 10g units (divide by 100 for kg).

---

## Quick Reference

### UUIDs
```
Service:   12345678-1234-1234-1234-123456789abc
Data:      87654321-4321-4321-4321-cba987654321  (NOTIFY)
Cmd:       11111111-2222-3333-4444-555555555555  (WRITE + NOTIFY)

Standard Battery Service: 0x180F
Battery Level Char:       0x2A19  (READ + NOTIFY, shows MIN of both batteries)
```

### Data Specs
| Property | Value |
|----------|-------|
| Channels | 8 (4 local + 4 remote) |
| Sample Rate | 1000 Hz |
| Packet Rate | ~100/sec |
| Samples/Packet | 1-10 |
| Bytes/Sample | 16 |
| Max Packet | 161 bytes |
| Value Range | int16 (-32768 to +32767) |

### Command Timeouts
| Type | Timeout |
|------|---------|
| Normal | 5 seconds |
| Calibration | 15 seconds |

---

## LED Control

The system includes WS2812 RGB LEDs on each Teensy device that provide visual status feedback. LEDs can be controlled via BLE commands.

### LED Status Indicators

When LEDs are enabled, they display different colors and patterns based on system state. **Colors are inverted when mock data is enabled** to clearly indicate mock data mode:

| State | Real Data Mode | Mock Data Mode | Pattern |
|-------|----------------|----------------|---------|
| **Booting** | Yellow | Yellow | Solid |
| **Idle** | Pink/Magenta (#A72468) | **Yellow** | Breathing effect |
| **Starting** | Dark Blue (#061C2F) | **Orange** | Slow pulse |
| **Running** | Dark Blue (#061C2F) | **Orange** | Solid |
| **Stopping** | Orange | Cyan | Fast blink |
| **Calibration Processing** | Green | **Magenta** | Solid |
| **Calibration Success** | Green | **Magenta** | Flash pattern |
| **Error** | Red | Red | Fast flash |

### LED Commands

All LED commands return standard JSON responses with `ok` status and round-trip time:

```json
// Turn on local LEDs
Send: LOCAL_LED_ON
Response: {"target":"LOCAL","cmd":"LED_ON","ok":true,"ms":45}

// Turn off remote LEDs
Send: REMOTE_LED_OFF
Response: {"target":"REMOTE","cmd":"LED_OFF","ok":true,"ms":52}

// Control both devices
Send: ALL_LED_ON
Response: {"target":"ALL","cmd":"LED_ON","ok":true,"ms":48}
```

### Usage Example

```python
# Python example
await client.write_gatt_char(CMD_UUID, b"ALL_LED_ON")  # Enable LEDs on both devices
await client.write_gatt_char(CMD_UUID, b"LOCAL_LED_OFF")  # Disable local LEDs only
```

**Note:** When LEDs are disabled (`LED_OFF`), all status indicators are turned off. When enabled (`LED_ON`), LEDs automatically display the current system status.

---

## Mock Data Generation

Mock data generation allows testing data rates and connection reliability without physical load cells. When enabled, the system generates synthetic waveforms instead of reading from the ADS1256 ADC.

### Wave Types

Each channel generates a different waveform pattern:

| Channel | Wave Type | Description |
|---------|-----------|-------------|
| 0 | Square Wave | Binary {-1, +1} pattern: `y = sign(sin(2πft))` |
| 1 | Triangle Wave | Linear ramp up/down: `y = (2/π) * arcsin(sin(2πft))` |
| 2 | Sawtooth Wave | Periodic reset pattern: `y = 2(tf - floor(tf + 0.5))` |
| 3 | Sine Wave | Smooth sinusoidal: `y = sin(2πft)` |

**Parameters:**
- Frequency: 1 Hz (base frequency)
- Amplitude: 100,000 counts (adjustable in code)
- Range: Repetitive, fixed-range patterns

### Usage

```python
# Enable mock data on both devices
await client.write_gatt_char(CMD_UUID, b"ALL_MOCK_ON")

# Start data acquisition with mock data
await client.write_gatt_char(CMD_UUID, b"ALL_START")

# Disable mock data to return to real hardware
await client.write_gatt_char(CMD_UUID, b"ALL_MOCK_OFF")
```

### Visual Indicators

When mock data is enabled, LEDs display **inverted colors** to clearly indicate mock data mode:

| State | Real Data Mode | Mock Data Mode |
|-------|----------------|----------------|
| **Idle** | Pink/Magenta breathing | **Yellow breathing** |
| **Starting** | Dark blue pulse | **Orange pulse** |
| **Running** | Dark blue solid | **Orange solid** |
| **Stopping** | Orange blink | Cyan blink |
| **Calibration** | Green | Magenta |

This visual distinction helps identify when the system is using mock data versus real hardware readings.

### Use Cases

- **Testing data rates**: Verify system can handle full data throughput
- **Connection reliability**: Test wireless communication without hardware
- **Development**: Develop and test software when load cells are unavailable
- **Debugging**: Isolate issues between hardware and software layers

**Note:** Mock data can be easily removed by deleting the `mock_data_generator.ino` file from the Teensy codebase.

---

## Battery Monitoring

The system has two batteries (local and remote). There are two ways to monitor battery status:

### 1. BAT Command (Detailed)

Send `BAT` to get full battery information:

```
Send: BAT
Response: {"target":"BLE","cmd":"BAT","ok":true,"local":{"v":4.12,"pct":85.0},"remote":{"v":3.98,"pct":72.0},"ms":0}
```

### 2. Standard BLE Battery Service (Quick)

The device also exposes a standard BLE Battery Service (0x180F) that any BLE diagnostic tool can read automatically:

- **Service UUID:** `0x180F`
- **Battery Level Characteristic:** `0x2A19`
- **Value:** Single byte 0-100%, representing the **minimum** of both batteries

This means if local is 85% and remote is 72%, the standard battery level will show 72% (worst-case).

---

## Troubleshooting

### No Sensor Data
1. Enable notifications on Data characteristic
2. Send `ALL_START` command
3. Check hardware connections

### No Command Response
1. Enable notifications on Cmd characteristic (same one you write to)
2. Check command spelling (case-insensitive)
3. Wait for timeout if device is unresponsive

### Connection Issues
- Server supports only 1 client at a time
- Server auto-restarts advertising after disconnect
- Reconnect and re-subscribe to notifications


--------

# Teensy 4.1 to ADS1256 Wiring Diagram

## Pin Connections

| ADS1256 Pin | Function | Teensy 4.1 Pin | Notes |
|-------------|----------|----------------|-------|
| VDD         | Power    | 3.3V           | ADS1256 power supply |
| AVDD        | Analog Power | 3.3V      | Analog power supply |
| DGND        | Digital Ground | GND      | Digital ground |
| AGND        | Analog Ground | GND       | Analog ground |
| SCLK        | SPI Clock | Pin 13 (SCK)  | SPI clock signal |
| DIN         | Data In   | Pin 11 (MOSI) | SPI data to ADS1256 |
| DOUT        | Data Out  | Pin 12 (MISO) | SPI data from ADS1256 |
| CS          | Chip Select | Pin 10      | SPI chip select (active low) |
| DRDY        | Data Ready | Pin 22       | Interrupt pin (active low) |
| RESET       | Reset     | Pin 8         | Reset pin (active low) |
| PDWN        | Power Down | 3.3V         | Tie high to keep powered |
| VREFP       | Positive Reference | External | Connect to your positive reference |
| VREFN       | Negative Reference | External | Connect to your negative reference |

## Changes from Teensy 3.1

- **CS Pin**: Changed from Pin 21 to Pin 10 (Teensy 4.1 default SPI CS)
- **All other pins remain the same**
- **Performance**: Teensy 4.1 runs at 600MHz vs 72MHz, providing much faster processing

## Power Supply Notes

- **ADS1256 operates at 3.3V** - Do NOT connect to 5V
- **Use clean power supply** - Consider adding decoupling capacitors (0.1µF ceramic + 10µF tantalum)
- **Separate analog and digital grounds** if possible for best noise performance

## Reference Voltage

- **VREFP/VREFN**: Determines the input voltage range
- **Common setup**: VREFP = +2.5V, VREFN = GND for 0-5V input range with gain
- **For ±2.5V inputs**: VREFP = +2.5V, VREFN = -2.5V

## Input Channels

The ADS1256 has 8 differential input channels:
- **AIN0-AIN7**: Analog input pins
- **AINCOM**: Common input for single-ended measurements

## Example Breadboard Layout

```
Teensy 4.1          ADS1256 Breakout
┌─────────────┐     ┌──────────────┐
│         3.3V├─────┤VDD      AIN0 │ ← Analog Input 0+
│          GND├─────┤DGND     AIN1 │ ← Analog Input 0-
│      Pin 13 ├─────┤SCLK     AIN2 │ ← Analog Input 1+
│      Pin 11 ├─────┤DIN      AIN3 │ ← Analog Input 1-
│      Pin 12 ├─────┤DOUT     AIN4 │ ← Analog Input 2+
│      Pin 10 ├─────┤CS       AIN5 │ ← Analog Input 2-
│      Pin 22 ├─────┤DRDY     AIN6 │ ← Analog Input 3+
│       Pin 8 ├─────┤RESET    AIN7 │ ← Analog Input 3-
│             │     │        VREFP│ ← +Reference (e.g., 2.5V)
│             │     │        VREFN│ ← -Reference (e.g., GND)
└─────────────┘     └──────────────┘
```

