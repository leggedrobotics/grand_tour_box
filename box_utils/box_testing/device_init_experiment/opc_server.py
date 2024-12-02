from flask import Flask, request, jsonify
from datetime import datetime
import csv

app = Flask(__name__)

restart_limit = 100
restart_count = 0
last_boot_id = None  # Track the last seen boot ID
date_string = datetime.now().strftime("%Y%m%d_%H%M%S")
csv_file = f"device_log_{date_string}.csv"

print("Restart limit:", restart_limit)

# Initialize CSV file with headers
with open(csv_file, mode="w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["Timestamp", "Attempt Number", "Boot ID", "Device List"])


@app.route("/restart", methods=["GET"])
def handle_restart():
    global restart_count, last_boot_id
    device_list = request.args.get("device_list", None)
    boot_id = request.args.get("boot_id", "unknown")  # Default to "unknown" if not provided

    # Increment restart count only if boot_id changes
    if boot_id != last_boot_id:
        restart_count += 1
        last_boot_id = boot_id

    # Log the device list, boot ID, timestamp, and attempt number to the CSV
    if device_list:
        with open(csv_file, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([datetime.now(), restart_count, boot_id, device_list])

    # Check if the restart limit is reached
    if restart_count <= restart_limit:
        return jsonify({"restart": True})
    else:
        return jsonify({"restart": False})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
