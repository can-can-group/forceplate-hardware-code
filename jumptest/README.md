# Jump Test – Force Plate BLE

HTML/JS app to connect to the force plate via BLE, stream left/right/total force on a live chart, save/load sessions, and run CMJ (countermovement jump) analysis.

## Requirements

- **Browser**: Web Bluetooth support (Chrome, Edge). The app must run in a **secure context** (HTTPS or `http://localhost`). Opening `index.html` via `file://` will not work for BLE.
- **Force plate**: ESP32 BLE server named `LoadCell_BLE_Server` (see Teensy41ADS1256 project).

## Running the app

1. Serve the folder over HTTP, for example:
   ```bash
   cd jumptest
   python -m http.server 8080
   ```
2. Open **http://localhost:8080** in the browser.
3. Click **Connect**, select the force plate, then use **ALL_START** to begin streaming. Left (local), Right (remote), and Total force appear on the chart.

## Commands

Use the buttons or custom command field to send BLE commands (e.g. `ALL_START`, `ALL_STOP`, `STATUS`, `BAT`, `LOCAL_CAL_TARE`, `LOCAL_CAL_SHOW`, `LOCAL_PING`, `REMOTE_PING`). Responses appear in the log.

## Save / Load

- **Save data**: Downloads a JSON file with `time`, `left_force`, `right_force`, `total_force` (and metadata). Values are in 10g units.
- **Load data**: Pick a previously saved JSON file. The chart and in-memory buffers are filled so you can view or re-analyse without re-recording.

## CMJ analysis

The app exports data in the format expected by the Python CMJ analyser (time in seconds, force in Newtons).

### Option 1: Export and run locally

1. Record or load a jump session.
2. Click **Export for CMJ** to download a JSON file (forces in N, key `index` = time in s).
3. Run the analyser:
   ```bash
   pip install -r requirements.txt
   python run_cmj_analysis.py cmj_export_XXXXXXXX.json
   ```
   Optional: write result to a file:
   ```bash
   python run_cmj_analysis.py cmj_export_XXXXXXXX.json -o result.json
   ```
   You can also pass a **saved** session file (10g units); the script converts to N and uses `time` as `index`.

### Option 2: Local analysis server (Run CMJ analysis button)

1. Start the server:
   ```bash
   pip install -r requirements.txt
   python server.py
   ```
2. In the app, record or load data, then click **Run CMJ analysis**. The app POSTs the current data to `http://localhost:8765/analyse` and shows the result (points, phases, jump height, RFD, etc.) in the **CMJ result** panel.

The server listens on port **8765** and accepts POST requests with JSON body: `index`, `left_force`, `right_force`, `total_force` (force in Newtons). It returns the full CMJ analysis JSON.

## File layout

- `index.html` – Single-page UI (connect, commands, chart, save/load, export, Run CMJ).
- `app.js` – BLE connection, packet parsing, chart updates, save/load, export, and analysis request.
- `cmj_analyser.py` – CMJ analysis logic (points, phases, jump height, RFD, etc.).
- `run_cmj_analysis.py` – CLI to run analysis on an exported or saved JSON file.
- `server.py` – Optional HTTP server for the “Run CMJ analysis” button (POST `/analyse`).
- `requirements.txt` – Python dependencies (pandas, numpy, scipy, flask).
