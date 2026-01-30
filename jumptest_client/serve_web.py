#!/usr/bin/env python3
"""
Serve the CMJ web app so you can open it on a tablet and use Web Bluetooth.
- Serves static files from web/ and POST /api/cmj (Python CMJ analyser).
- Use --https so the tablet can use https://<this-PC-IP>:8443 (required for Web Bluetooth).
- Install PyOpenSSL for HTTPS: pip install pyopenssl
"""

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

_root = Path(__file__).resolve().parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

from flask import Flask, request, jsonify, send_from_directory

# Add parent for config/force_plate/analysers
sys.path.insert(0, str(_root))

from analysers import CMJ_Analyser

app = Flask(__name__, static_folder=str(_root / "web"), static_url_path="")
WEB_DIR = _root / "web"

# Recordings folder at project root (parent of jumptest_client)
RECORDINGS_DIR = _root.parent / "recordings"
RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)


def make_json_serializable(obj):
    """Convert numpy/pandas types to native Python for JSON serialization."""
    import numpy as np
    # Must check numpy scalars BEFORE native int/float (numpy.int64 is isinstance int)
    if hasattr(obj, "item") and hasattr(obj, "dtype"):
        return obj.item()
    if isinstance(obj, (np.integer, np.floating)):
        return obj.item()
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, dict):
        return {k: make_json_serializable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [make_json_serializable(v) for v in obj]
    if isinstance(obj, (int, float, str, bool, type(None))):
        return obj
    if hasattr(obj, "__iter__") and not isinstance(obj, (str, bytes)):
        return [make_json_serializable(v) for v in obj]
    return obj


@app.route("/")
def index():
    return send_from_directory(WEB_DIR, "index.html")


@app.route("/favicon.ico")
def favicon():
    return "", 404


@app.route("/app.js")
def app_js():
    return send_from_directory(WEB_DIR, "app.js")


@app.route("/style.css")
def style_css():
    return send_from_directory(WEB_DIR, "style.css")


@app.route("/api/cmj", methods=["POST"])
def api_cmj():
    """Expects JSON { left_force, right_force, total_force, index }. Returns CMJ analysis."""
    try:
        data = request.get_json(force=True)
        if not data:
            return jsonify({"error": "Missing JSON body"}), 400
        for key in ("left_force", "right_force", "total_force", "index"):
            if key not in data:
                return jsonify({"error": f"Missing key: {key}"}), 400
        result = CMJ_Analyser.compute_cmj_test(data)
        result = make_json_serializable(result)
        return jsonify(result)
    except ValueError as e:
        return jsonify({"error": str(e)}), 400
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/save", methods=["POST"])
def api_save():
    """Save recorded samples to a JSON file. Body: { samples: [[8 ints], ...], name?: string }."""
    try:
        data = request.get_json(force=True)
        samples = data.get("samples")
        if not samples or not isinstance(samples, list):
            return jsonify({"error": "Missing or invalid 'samples' array"}), 400
        name = (data.get("name") or "").strip()
        if not name:
            name = "recording_{}.json".format(datetime.now().strftime("%Y%m%d_%H%M%S"))
        if not name.endswith(".json"):
            name += ".json"
        if "/" in name or "\\" in name:
            return jsonify({"error": "Invalid filename"}), 400
        path = RECORDINGS_DIR / name
        payload = {
            "samples": samples,
            "name": name,
            "saved_at": datetime.now().isoformat(),
            "sample_count": len(samples),
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=0)
        return jsonify({"ok": True, "name": name, "sample_count": len(samples)})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/recordings", methods=["GET"])
def api_recordings():
    """List saved recording filenames with metadata."""
    try:
        out = []
        for p in sorted(RECORDINGS_DIR.glob("*.json"), key=lambda x: x.stat().st_mtime, reverse=True):
            try:
                with open(p, "r", encoding="utf-8") as f:
                    meta = json.load(f)
                out.append({
                    "name": p.name,
                    "saved_at": meta.get("saved_at", ""),
                    "sample_count": meta.get("sample_count", 0),
                })
            except Exception:
                out.append({"name": p.name, "saved_at": "", "sample_count": 0})
        return jsonify({"recordings": out})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/load", methods=["GET"])
def api_load():
    """Load a recording by name. Query: name=filename.json"""
    name = (request.args.get("name") or "").strip()
    if not name or "/" in name or "\\" in name:
        return jsonify({"error": "Invalid or missing name"}), 400
    path = RECORDINGS_DIR / name
    if not path.is_file():
        return jsonify({"error": "File not found"}), 404
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        samples = data.get("samples", [])
        return jsonify({"samples": samples, "name": name, "sample_count": len(samples)})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


def main():
    parser = argparse.ArgumentParser(description="Serve CMJ web app for tablet + Web Bluetooth")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default 0.0.0.0)")
    parser.add_argument("--port", type=int, default=None, help="Port (default 5000 for HTTP, 8443 for HTTPS)")
    parser.add_argument("--https", action="store_true", help="Use HTTPS (required for Web Bluetooth on tablet). Needs PyOpenSSL.")
    args = parser.parse_args()

    port = args.port if args.port is not None else (8443 if args.https else 5000)

    if args.https:
        print("Serving at https://{}:{}".format(args.host, port))
        print("On tablet: open https://<this-PC-IP>:{} and accept the certificate warning.".format(port))
        try:
            app.run(host=args.host, port=port, ssl_context="adhoc", threaded=True)
        except Exception as e:
            if "adhoc" in str(e).lower() or "pyopenssl" in str(e).lower():
                print("HTTPS requires PyOpenSSL: pip install pyopenssl")
            else:
                print(e)
            sys.exit(1)
    else:
        print("Serving at http://{}:{}".format(args.host, port))
        print("For tablet + Web Bluetooth use: python serve_web.py --https")
        app.run(host=args.host, port=port, threaded=True)


if __name__ == "__main__":
    main()
