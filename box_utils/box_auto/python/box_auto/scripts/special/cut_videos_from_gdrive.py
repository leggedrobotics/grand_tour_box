import sys
import os
import subprocess
import gdown
from typing import List, Tuple
from urllib.parse import unquote


def get_filename_from_gdrive_url(url: str) -> str:
    """
    Attempt to extract the original filename from a Google Drive URL or response.
    Returns a sanitized filename.
    """
    try:
        # First try to get filename from URL if it's present
        if "file/d/" in url:
            # Some Google Drive URLs include the filename after /view?usp=sharing
            parts = url.split("/")
            if len(parts) > 5:
                potential_filename = unquote(parts[5].split("?")[0])
                if potential_filename and potential_filename != "view":
                    return potential_filename

        # If we couldn't get the filename from the URL, generate a random one
        import uuid

        return f"video_{str(uuid.uuid4())[:8]}"
    except Exception as e:
        print(f"Error extracting filename: {e}", file=sys.stderr)
        return f"video_{str(uuid.uuid4())[:8]}"


def download_from_gdrive(url: str, output_path: str) -> bool:
    """
    Download a file from Google Drive.

    Args:
        url (str): Google Drive URL
        output_path (str): Where to save the downloaded file

    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Extract file ID from Google Drive URL
        file_id = None
        if "/file/d/" in url:
            file_id = url.split("/file/d/")[1].split("/")[0]
        elif "id=" in url:
            file_id = url.split("id=")[1].split("&")[0]

        if not file_id:
            print(f"Invalid Google Drive URL: {url}")
            return False

        # Download the file
        gdown.download(id=file_id, output=output_path, quiet=False)
        return os.path.exists(output_path)
    except Exception as e:
        print(f"Error downloading file: {e}", file=sys.stderr)
        return False


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
    cmd = ["ffmpeg", "-i", input_path, "-ss", start_time, "-to", end_time, "-c", "copy", output_path]

    try:
        # Run ffmpeg command
        subprocess.run(cmd, check=True)
        print(f"Successfully created: {output_path}")
    except subprocess.CalledProcessError as e:
        print(f"Error during video cutting: {e}", file=sys.stderr)
        sys.exit(1)


def process_video_list(video_list: List[Tuple[str, str, str, str]], output_dir: str = "videos") -> None:
    """
    Process a list of videos: download each one and cut according to timestamps.

    Args:
        video_list: List of tuples containing (url, start_time, end_time, filename)
        output_dir: Directory to store downloaded and processed videos
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    for i, (start_time, end_time, url, custom_filename) in enumerate(video_list, 1):
        print(f"\nProcessing video {i}/{len(video_list)}")

        # Generate output filename
        if custom_filename:
            # Use provided filename if available
            base_filename = custom_filename
        else:
            # Try to get filename from URL
            base_filename = get_filename_from_gdrive_url(url)

        # Ensure the filename has a video extension
        if not base_filename.lower().endswith((".mp4", ".mkv", ".avi", ".mov")):
            base_filename += ".mp4"

        output_path = os.path.join(output_dir, base_filename)

        # Download the video
        print(f"Downloading video from: {url}")
        if not download_from_gdrive(url, output_path):
            print(f"Skipping video {i} due to download failure")
            continue

        # Cut the video
        print(f"Cutting video from {start_time} to {end_time}")
        cut_video(output_path, start_time, end_time)

        # Remove the original downloaded file
        os.remove(output_path)
        print(f"Removed original uncut video: {output_path}")


if __name__ == "__main__":
    # Example usage

    # https://drive.google.com/file/d/1Bbd_JSAJlaz51b4Q01PMWef4tvBluQTh/view?usp=sharing  # to be added
    video_list = [
        (
            "00:00:00",
            "00:00:06",
            "https://drive.google.com/file/d/1pVHCtuE_feS72zluMSJTKQ1OVw0Ufy7a/view?usp=sharing",
            "GX020731.mp4",
        ),
        (
            "00:01:46",
            "00:01:50",
            "https://drive.google.com/file/d/1pVHCtuE_feS72zluMSJTKQ1OVw0Ufy7a/view?usp=sharing",
            "GX020731_2.mp4",
        ),
        (
            "00:01:55",
            "00:02:05",
            "https://drive.google.com/file/d/1A4KMEABWLdfZ_QjKVCxf13xrFJdgQJWc/view?usp=sharing",
            "GX010730.mp4",
        ),
        (
            "00:10:55",
            "00:11:04",
            "https://drive.google.com/file/d/1A4KMEABWLdfZ_QjKVCxf13xrFJdgQJWc/view?usp=sharing",
            "GX010730_2.mp4",
        ),
        (
            "00:11:34",
            "00:11:43",
            "https://drive.google.com/file/d/1s5pSIQEcMejOJ6soBodK9-WVfflFE4na/view?usp=sharing",
            "GX010731.mp4",
        ),
        (
            "00:00:19",
            "00:00:28",
            "https://drive.google.com/file/d/1b2Voe92vjvtqWagQgZJtqIqbr_pLdzn0/view?usp=sharing",
            "GX010727.mp4",
        ),
        (
            "00:01:55",
            "00:02:03",
            "https://drive.google.com/file/d/1RHrwFEQ9cIw6LlNtyLufY0RXt8_LPpI7/view?usp=sharing",
            "GX010714.mp4",
        ),
        (
            "00:03:24",
            "00:03:33",
            "https://drive.google.com/file/d/1RHrwFEQ9cIw6LlNtyLufY0RXt8_LPpI7/view?usp=sharing",
            "GX010714_2.mp4",
        ),
        (
            "00:01:03",
            "00:01:15",
            "https://drive.google.com/file/d/1bSs6gSYJHYzZZYZoPhl8_XG7Z0AiVHlH/view?usp=sharing",
            "GX010710.mp4",
        ),
        (
            "00:01:44",
            "00:02:00",
            "https://drive.google.com/file/d/1bSs6gSYJHYzZZYZoPhl8_XG7Z0AiVHlH/view?usp=sharing",
            "GX010710_2.mp4",
        ),
        (
            "00:00:38",
            "00:00:47",
            "https://drive.google.com/file/d/1hsHSHrJUjor9wIYnf4XOACravEEE5ZRe/view?usp=sharing",
            "GX010705.mp4",
        ),
        (
            "00:02:17",
            "00:02:25",
            "https://drive.google.com/file/d/19zgYfQUm_arVgnxMeoDEAYBCWL1fjBXE/view?usp=sharing",
            "GX010704.mp4",
        ),
        (
            "00:01:45",
            "00:01:53",
            "https://drive.google.com/file/d/1UsaJupw3mfJOPU9nC1jh2zIgf_MFIxMB/view?usp=sharing",
            "GX010686.mp4",
        ),
        (
            "00:03:45",
            "00:04:00",
            "https://drive.google.com/file/d/1UsaJupw3mfJOPU9nC1jh2zIgf_MFIxMB/view?usp=sharing",
            "GX010686_2.mp4",
        ),
        (
            "00:03:03",
            "00:03:25",
            "https://drive.google.com/file/d/1I4T54RWdGJuavbijdf9c-V0pLqKVEFW_/view?usp=sharing",
            "GX010681.mp4",
        ),
        (
            "00:05:28",
            "00:05:42",
            "https://drive.google.com/file/d/1dT218CKCmvNo3HNKsvz-IqWA6oqU05-X/view?usp=sharing",
            "GX010680.mp4",
        ),
        (
            "00:00:07",
            "00:00:23",
            "https://drive.google.com/file/d/1rPwcVe4X2OnuCoY3HVrRbzYqeGEvEExa/view?usp=sharing",
            "GX010679.mp4",
        ),
        (
            "00:00:21",
            "00:00:30",
            "https://drive.google.com/file/d/1dVbvQxU7Zfe5NjeaLzh1VmurRxOohuR6/view?usp=sharing",
            "GX010672.mp4",
        ),
        (
            "00:03:54",
            "00:04:04",
            "https://drive.google.com/file/d/1ZpDxx03SU4D6CDtHVu2BW5_682tGGzZb/view?usp=sharing",
            "GX010670.mp4",
        ),
        (
            "00:00:10",
            "00:00:27",
            "https://drive.google.com/file/d/1d_vOB0eYTuzIblSOgvKIlrhY9atmJ4Q-/view?usp=sharing",
            "GX010674.mp4",
        ),
        (
            "00:00:15",
            "00:00:30",
            "https://drive.google.com/file/d/1t1-1eUfwJNPCigaBuwBterzrY9H5dPk1/view?usp=sharing",
            " GX010677.mp4",
        ),
        (
            "00:00:15",
            "00:00:22",
            "https://drive.google.com/file/d/1eg2kdXkFTasFf4r9cU4FNMTlDPQkAPZg/view?usp=sharing",
            "GX010676.mp4",
        ),
        (
            "00:03:35",
            "00:03:50",
            "https://drive.google.com/file/d/1s5pSIQEcMejOJ6soBodK9-WVfflFE4na/view?usp=sharing",
            "GX010731_2.mp4",
        ),
        (
            "00:08:38",
            "00:08:44",
            "https://drive.google.com/file/d/1nD11lqJQ3WMJtBFUG6U2ZUkl4DUFhKGp/view?usp=sharing",
            "GX010667.mp4",
        ),
        (
            "00:01:07",
            "00:01:20",
            "https://drive.google.com/file/d/10Fomb1Tu4i2KqBHLdwmmRCmL6aCTlXVF/view?usp=sharing",
            "GX010663.mp4",
        ),
        (
            "00:00:28",
            "00:00:37",
            "https://drive.google.com/file/d/13GIWV_2CQElNEgJLeLT0Fq81neBXnKyw/view?usp=sharing",
            "GX010661.mp4",
        ),
        (
            "00:01:28",
            "00:01:38",
            "https://drive.google.com/file/d/1Me7cERbVQ9elfmXnEqA1NS7viAZchPTk/view?usp=sharing",
            "GX010657.mp4",
        ),
        (
            "00:00:15",
            "00:00:25",
            "https://drive.google.com/file/d/1oS4GoYb6L1509vBzLObsIYCsDSJ7I9XZ/view?usp=sharing",
            "GX010655.mp4",
        ),
        (
            "00:00:35",
            "00:00:50",
            "https://drive.google.com/file/d/14G_UutE0zi8C9S7VdBKaPmYnxMrjU-m1/view?usp=sharing",
            "GX010654.mp4",
        ),
        (
            "00:01:38",
            "00:01:58",
            "https://drive.google.com/file/d/14JPj9PBQhBUcOJJCXx3fcvj43aYaoN-4/view?usp=sharing",
            " GX010653.mp4",
        ),
        (
            "00:00:01",
            "00:00:41",
            "https://drive.google.com/file/d/1OJkIyqcuqN98IAf18Am0oHzTFnNy2gQx/view?usp=sharing",
            "GX010648.mp4",
        ),
        (
            "00:00:10",
            "00:00:20",
            "https://drive.google.com/file/d/1oj-0_ZuerBM0-XOeCX9MGuXNZofIOFFX/view?usp=sharing",
            "GX010647.mp4",
        ),
        (
            "00:00:20",
            "00:00:30",
            "https://drive.google.com/file/d/1p7aW8JsIgrDYC1b1CkYyCa5uxUR2sktI/view?usp=sharing",
            "GX010646.mp4",
        ),
        (
            "00:01:00",
            "00:01:10",
            "https://drive.google.com/file/d/1choGfdLuqjQrDUw3ZyXMZo_RDAAqCm8l/view?usp=sharing",
            "GX010633.mp4",
        ),
        (
            "00:02:06",
            "00:02:30",
            "https://drive.google.com/file/d/1AePeHq7_CH-EDpSCu4yj2GrwUOYK3q-t/view?usp=sharing",
            "GX010632.mp4",
        ),
        (
            "00:00:05",
            "00:00:20",
            "https://drive.google.com/file/d/16pPPKLy2OfMMagmjpj7hiWiAoH-EIy7r/view?usp=sharing",
            "jonas_recording_00_2836.mov",
        ),
        (
            "00:00:14",
            "00:00:24",
            "https://drive.google.com/file/d/1ZweF2KbrL1gb1JvlFKJ2qt2KuGnkAHmW/view?usp=sharing",
            "jonas_recording_02_2838.mov",
        ),
        (
            "00:00:05",
            "00:00:13",
            "https://drive.google.com/file/d/1F8dPu27EDsDhhIapxpJoVYB1CLKkXdb8/view?usp=sharing",
            "IMG_2790.mov",
        ),
        (
            "00:00:28",
            "00:00:35",
            "https://drive.google.com/file/d/18DkG3jqCtohVW9R8DN2Yjakl8yB0tDAI/view?usp=sharing",
            "IMG_2748.mov",
        ),
        (
            "00:00:13",
            "00:00:31",
            "https://drive.google.com/file/d/1SVc3rWTgWwuz1wg9h3bOZdnoZpeub5xH/view?usp=sharing",
            "IMG_2704.mov",
        ),
        (
            "00:00:00",
            "00:00:15",
            "https://drive.google.com/file/d/1hVhuH41MfhiIpsTHYicRpn2LeAXZCaqb/view?usp=sharing",
            "IMG_2711.mov",
        ),
        (
            "00:01:48",
            "00:02:00",
            "https://drive.google.com/file/d/1cGNq7ZaYoXax8GYICWbvEN0jC9HKtRxq/view?usp=sharing",
            "IMG_2680.mov",
        ),
        (
            "00:01:10",
            "00:01:25",
            "https://drive.google.com/file/d/1Ggd49iICw6AIGfMan29wCb6WgC4RmfRd/view?usp=sharing",
            "GX010625.mp4",
        ),
        (
            "00:00:24",
            "00:00:34",
            "https://drive.google.com/file/d/1Bh-3qMNGpMUby5ic-kDJhsMtIz57xISc/view?usp=sharing",
            "GX010624.mp4",
        ),
        (
            "00:06:55",
            "00:07:05",
            "https://drive.google.com/file/d/1r7d3z9d63vV0Y6hV-Fj3Tx6wr2dcGL17/view?usp=sharing",
            "GX010624_2.mp4",
        ),
        (
            "00:03:59",
            "00:04:02",
            "https://drive.google.com/file/d/1OLX_pTzJHPAbpua4irCIZ-S20qZR9OM_/view?usp=sharing",
            "GX010619.mp4",
        ),
        (
            "00:02:10",
            "00:02:30",
            "https://drive.google.com/file/d/1mXyU58aZnnU4ZIWwafq2rWcN2gf4B4tK/view?usp=sharing",
            "GX010606.mp4",
        ),
        (
            "00:00:10",
            "00:00:22",
            "https://drive.google.com/file/d/1dvyETjdmfLBTB8HOFzo_NRxQV0U6_-2e/view?usp=sharing",
            "GX010603.mp4",
        ),
        (
            "00:00:00",
            "00:00:10",
            "https://drive.google.com/file/d/1_PhojPgywA9df_cBa_0ax5dosKLDW1Bq/view?usp=sharing",
            "GX010600.mp4",
        ),
        (
            "00:00:10",
            "00:00:25",
            "https://drive.google.com/file/d/1vEAX5A4Lx_PekcoqTrqRN86p64EkmRhG/view?usp=sharing",
            "IMG_8491.mp4",
        ),
        (
            "00:00:26",
            "00:00:42",
            "https://drive.google.com/file/d/1gyGYO1pxw7xnn07mMBOUCnTmsCp6Jzw9/view?usp=sharing",
            "IMG_8635.mp4",
        ),
    ]

    process_video_list(video_list)
