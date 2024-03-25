#!/bin/bash

timeout=60 # Seconds
endtime=$(($(date +%s) + timeout))

while [ $(date +%s) -lt $endtime ]; do
    # Check journal to see if a foreign ptp time master has been found.
    if journalctl -u ptp4l_enp45s0.service -b | grep "new foreign master"; then
        echo "Found new foreign time master."
        exit 0
    fi
    sleep 1
done

echo "Timeout reached without finding a new foreign time master."
exit 1