"""
Local HTTP server for CMJ analysis.
Same behaviour as run_cmj_analysis.py:
  POST /analyse with JSON: "index" or "time", "left_force", "right_force", "total_force".
  If "meta" is present, forces are in 10g units and are converted to Newtons (v/100 * 9.81).
  Otherwise forces are expected in Newtons.
"""
import json
import sys
from http.server import HTTPServer, SimpleHTTPRequestHandler

try:
    from flask import Flask, request, jsonify
    HAS_FLASK = True
except ImportError:
    HAS_FLASK = False

from cmj_analyser import CMJ_Analyser

PORT = 8765
ANALYSE_PATH = "/analyse"
G = 9.81


def prepare_cmj_input(data):
    """Same prep as run_cmj_analysis.py: index from time, then convert forces if meta present."""
    if "index" not in data and "time" in data:
        data["index"] = data["time"]
    if "meta" in data and "left_force" in data:
        data["left_force"] = [(float(v) / 100.0) * G for v in data["left_force"]]
        data["right_force"] = [(float(v) / 100.0) * G for v in data["right_force"]]
        data["total_force"] = [(float(v) / 100.0) * G for v in data["total_force"]]
    return data


def run_flask():
    app = Flask(__name__)

    @app.route(ANALYSE_PATH, methods=["POST"])
    def analyse():
        try:
            data = request.get_json(force=True)
            if not data:
                return jsonify({"ok": False, "error": "No JSON body"}), 400
            prepare_cmj_input(data)
            result = CMJ_Analyser.compute_cmj_test(data)
            return jsonify(result)
        except ValueError as e:
            return jsonify({"ok": False, "error": str(e)}), 400
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 500

    # CORS for local dev (app may be served from file or another port)
    @app.after_request
    def cors(resp):
        resp.headers["Access-Control-Allow-Origin"] = "*"
        resp.headers["Access-Control-Allow-Methods"] = "POST, OPTIONS"
        resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
        return resp

    @app.route(ANALYSE_PATH, methods=["OPTIONS"])
    def options():
        return "", 204

    print("CMJ analysis server at http://localhost:%s%s" % (PORT, ANALYSE_PATH))
    print("POST JSON: index (or time), left_force, right_force, total_force. Include meta for 10g→N conversion.")
    app.run(host="0.0.0.0", port=PORT, debug=False, threaded=True)


def run_stdlib():
    class Handler(SimpleHTTPRequestHandler):
        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_POST(self):
            if self.path != ANALYSE_PATH:
                self.send_response(404)
                self.end_headers()
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length)
                data = json.loads(body.decode("utf-8"))
                prepare_cmj_input(data)
                result = CMJ_Analyser.compute_cmj_test(data)
                out = json.dumps(result).encode("utf-8")
            except Exception as e:
                self.send_response(400)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"ok": False, "error": str(e)}).encode("utf-8"))
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(out)

    print("CMJ analysis server at http://localhost:%s%s" % (PORT, ANALYSE_PATH))
    print("POST JSON: index (or time), left_force, right_force, total_force. Include meta for 10g→N conversion.")
    server = HTTPServer(("0.0.0.0", PORT), Handler)
    server.serve_forever()


if __name__ == "__main__":
    if HAS_FLASK:
        run_flask()
    else:
        run_stdlib()
