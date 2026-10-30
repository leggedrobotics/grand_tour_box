from pathlib import Path


def prepare_recording_directory(path):
    """Create the selected destination; never silently record on another disk."""
    directory = Path(path).expanduser()
    try:
        directory.mkdir(parents=True, exist_ok=True)
    except OSError as error:
        raise OSError(f"Cannot prepare recording directory {directory}: {error}") from error
    return str(directory)
