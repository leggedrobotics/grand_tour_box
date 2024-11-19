#!/usr/bin/env python3

import sys
import os
import subprocess


def cut_video(input_path, start_time, end_time):
    """
    Cut a video file using ffmpeg without re-encoding.

    Args:
        input_path (str): Path to input video file
        start_time (str): Start time in format HH:MM:SS or MM:SS
        end_time (str): End time in format HH:MM:SS or MM:SS
    """
    # Get the directory and filename
    directory = os.path.dirname(input_path)
    filename = os.path.basename(input_path)
    name, ext = os.path.splitext(filename)

    # Create output filename
    output_path = os.path.join(directory, f"{name}_cut{ext}")

    # Construct ffmpeg command
    # -ss: start time
    # -to: end time
    # -c copy: copy streams without re-encoding
    cmd = ["ffmpeg", "-i", input_path, "-ss", start_time, "-to", end_time, "-c", "copy", output_path]

    try:
        # Run ffmpeg command
        subprocess.run(cmd, check=True)
        print(f"Successfully created: {output_path}")
    except subprocess.CalledProcessError as e:
        print(f"Error during video cutting: {e}", file=sys.stderr)
        sys.exit(1)


def main():
    # Check if correct number of arguments is provided
    if len(sys.argv) != 4:
        print("Usage: python3 cut_video.py <input_video> <start_time format> <end_time>")
        print("Time format: HH:MM:SS or MM:SS")
        print("Example: python3 cut_video.py test.mp4 00:10 00:20")
        sys.exit(1)

    input_path = sys.argv[1]
    start_time = sys.argv[2]
    end_time = sys.argv[3]

    # Check if input file exists
    if not os.path.exists(input_path):
        print(f"Error: Input file '{input_path}' does not exist", file=sys.stderr)
        sys.exit(1)

    cut_video(input_path, start_time, end_time)


if __name__ == "__main__":
    main()
