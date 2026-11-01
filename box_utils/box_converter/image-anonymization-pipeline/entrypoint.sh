#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash" --
if [[ -n "${KLEINKRAM_CONFIG:-}" ]]; then
    umask 077
    printf '%s\n' "$KLEINKRAM_CONFIG" > "$HOME/.kleinkram.json"
fi

exec "$@"
