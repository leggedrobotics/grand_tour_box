#!/bin/bash

# Use the current directory for the output file
output_file="/tmp/clock_info.txt"

> "$output_file"
while true; do
    # Initialize an empty array to store the output
    declare -a output_lines

    # Fetch all outputs
    for src in arm core h264 isp v3d uart pwm emmc pixel vec hdmi dpi; do
        if ! vcgencmd measure_clock $src &> /dev/null; then
            output_lines+=("$src:\tNot available")
        else
            output_lines+=("$src:\t$(vcgencmd measure_clock $src)")
        fi
    done

    truncate -s 0  "$output_file"
    # Write all lines to the file at once
    printf '%s\n' "${output_lines[@]}" > "$output_file"
    output_lines=()
    # Sleep for 100ms
    sleep 0.1
done