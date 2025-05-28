#!/bin/bash

# Ensure the script is run as root
if [ "$EUID" -ne 0 ]; then 
  echo "Please run this script with sudo."
  exit 1
fi

# Set duration for monitoring (in seconds)
DURATION=120 #${1:-10}  # Default to 1800 seconds (30 minutes) if no argument is provided

# Get the current date in YYYY-MM-DD format
CURRENT_DATE=$(date +%Y-%m-%d)

# Create a directory to store the benchmark logs
OUTPUT_DIR="/data/${CURRENT_DATE}_benchmark"
mkdir -p "$OUTPUT_DIR"

echo "Logs will be saved in: $OUTPUT_DIR"

# Network profiling for interface enp45s0 (run in background)
sudo iftop -t -i mgbe0 > "$OUTPUT_DIR/network_usage_jetson_mgbe0.txt" &
IFTOP_PID1=$!
echo "Started network monitoring on mgbe0 (PID: $IFTOP_PID1)"

# CPU usage profiling using sar
sar -u 1 "$DURATION" > "$OUTPUT_DIR/cpu_usage_jetson.log" &
echo "Started CPU usage monitoring"

# Disk write speed profiling using iostat
iostat -dx nvme0n1p1 1 "$DURATION" > "$OUTPUT_DIR/disk_write_speed_jetson_nvme0n1p1.log" &
echo "Started disk write speed monitoring for nvme0n1p1"

sleep $DURATION

sudo kill -9 "$IFTOP_PID1"
sudo kill -9 "$IFTOP_PID2"

sleep 1
wait

echo "Benchmarking completed. Logs saved in: $OUTPUT_DIR"