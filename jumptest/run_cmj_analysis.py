#!/usr/bin/env python3
"""
Read a CMJ export JSON file and run CMJ_Analyser.compute_cmj_test.
Output the result as JSON to stdout or to an output file.
"""
import json
import sys
import argparse
from cmj_analyser import CMJ_Analyser


def main():
    parser = argparse.ArgumentParser(description="Run CMJ analysis on exported JSON.")
    parser.add_argument("input", help="Path to JSON file (from 'Export for CMJ')")
    parser.add_argument("-o", "--output", help="Write result to file (default: stdout)")
    args = parser.parse_args()

    with open(args.input, "r", encoding="utf-8") as f:
        data = json.load(f)

    if "index" not in data:
        if "time" in data:
            data["index"] = data["time"]
        else:
            print("Error: JSON must contain 'index' or 'time' and force arrays.", file=sys.stderr)
            sys.exit(1)

    # If data has "meta" it's likely a save file (10g units). Convert to N for analyser.
    if "meta" in data and "left_force" in data:
        g = 9.81
        data["left_force"] = [(v / 100.0) * g for v in data["left_force"]]
        data["right_force"] = [(v / 100.0) * g for v in data["right_force"]]
        data["total_force"] = [(v / 100.0) * g for v in data["total_force"]]

    try:
        result = CMJ_Analyser.compute_cmj_test(data)
    except Exception as e:
        print("Analysis error:", e, file=sys.stderr)
        sys.exit(1)

    out = json.dumps(result, indent=2)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(out)
        print("Wrote", args.output)
    else:
        print(out)


if __name__ == "__main__":
    main()
