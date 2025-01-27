import pathlib
import os
from box_auto.utils import get_bag, create_github_issue
from matplotlib import pyplot as plt
from box_auto.scripts.verification.topic_freq import read_rosbag_and_generate_histograms


def analyze_bag_and_report(bag_file: str, output_dir: str):
    """
    Analyze a ROS bag file, generate histograms, and create a GitHub issue with results.

    Args:
        bag_file (str): Path to the ROS bag file.
        output_dir (str): Directory to save output files.
    """
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Extract the bag's name for labeling
    bag_name = pathlib.Path(bag_file).stem

    # Run frequency analysis and generate histograms
    output_image = os.path.join(output_dir, f"{bag_name}.png")
    output_text_file = os.path.join(output_dir, "freq.txt")
    read_rosbag_and_generate_histograms(bag_file, output_dir, bag_name)

    # Read the results from the frequency text file
    with open(output_text_file, "r") as f:
        results_summary = f.read()

    # Create a GitHub issue with the results and the histogram image
    issue_body = (
        f"## Frequency Analysis Results for `{bag_name}`\n\n"
        f"### Summary:\n```\n{results_summary}\n```\n"
        f"### Histogram:\n![{bag_name}](attachment:{output_image})"
    )
    create_github_issue(
        title=f"Frequency Analysis Report: {bag_name}",
        body=issue_body,
        label="test_auto_verification",
        images=[output_image]
    )


if __name__ == "__main__":
    # Get the bag file matching the pattern
    pattern = "*_jetson_hdr_front.bag"
    try:
        input_bag = get_bag(pattern)
        print(f"Found bag file: {input_bag}")

        # Set output directory
        output_dir = os.path.join("verification", pathlib.Path(input_bag).stem)

        # Run analysis and report results
        analyze_bag_and_report(input_bag, output_dir)

    except FileNotFoundError as e:
        print(f"Error: {e}")
