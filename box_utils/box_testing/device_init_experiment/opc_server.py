from flask import Flask, request, jsonify
from datetime import datetime
import csv

app = Flask(__name__)

restart_limit = 5
restart_count = 0
csv_file = "device_log.csv"

# Initialize CSV file with headers
with open(csv_file, mode="w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Timestamp", "Attempt Number", "Device List"])


@app.route("/restart", methods=["GET"])
def handle_restart():
    global restart_count
    device_list = request.args.get("device_list", None)

    # Log the device list, timestamp, and attempt number to the CSV
    if device_list:
        with open(csv_file, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([datetime.now(), restart_count, device_list])

    if restart_count < restart_limit:
        restart_count += 1
        return jsonify({"restart": True})
    else:
        return jsonify({"restart": False})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
