# CMJ Web Client (Web Bluetooth)

Single-page app for tablet: connect to the force plate via **Web Bluetooth**, record, and run CMJ analysis (backend runs the Python analyser).

## Requirements

- **Chrome** on Android/tablet (Web Bluetooth is supported there).
- **HTTPS** or **localhost** (browser requires a secure context for Web Bluetooth).
- PC and tablet on the same network when using the tablet.

## Run the server (on your PC)

From `jumptest_client`:

```bash
# Install deps (including Flask + PyOpenSSL for HTTPS)
pip install -r requirements.txt

# For tablet: use HTTPS (required for Web Bluetooth)
python serve_web.py --https
```

Then on the **tablet** (Chrome):

1. Find your PC’s IP (e.g. `192.168.1.10`).
2. Open **https://192.168.1.10:8443**.
3. Accept the certificate warning (self-signed).
4. Tap **Connect to force plate** → choose the LoadCell device.
5. **Start recording** → do the jump → **Stop recording** → **Run CMJ**.

## Local only (no tablet)

```bash
python serve_web.py
# Open http://localhost:5000
```

Web Bluetooth still needs a secure context, so use Chrome and open **https://localhost:5000** if your setup allows, or keep using the desktop app (`run_cmj.py`).

## Options

- `--host 0.0.0.0` – listen on all interfaces (default).
- `--port 8443` – port (default 8443 with `--https`, 5000 otherwise).
- `--https` – serve over HTTPS (needs PyOpenSSL; required for tablet + Web Bluetooth).
