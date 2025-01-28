#!/bin/bash
# Wrapper script to set environment variables that kleinkram actions api uses, to mock the cloud behaviour locally.

# Define your environment variables
MISSION_UUID="f925eb38-92c3-4318-9ebb-712e43424589" # 2024-11-02-17-43-10
ACTION_UUID="123-xyz" # Dummy action uuid
PROJECT_UUID="226223c3-8593-4fb8-8e41-cc5d23e6ccb7" # Grand Tour Kappi Playground
LOCAL_MISSION_DATA="/home/kappi/rsl/boxi/empty" # Empty directory


# Call the original script with the environment variables passed as --env
./run.sh \
    --type=kleinkram \
    --debug \
    --mission-data="$LOCAL_MISSION_DATA" \
    --env="MISSION_UUID=$MISSION_UUID" \
    --env="ACTION_UUID=$ACTION_UUID" \
    --env="PROJECT_UUID=$PROJECT_UUID" \
    "$@"
