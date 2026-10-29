"""Publish complete local outputs without exposing partial files to later stages."""

from contextlib import contextmanager
import os
from pathlib import Path
import tempfile


@contextmanager
def atomic_output(destination, overwrite=True):
    """Yield a temporary path next to destination and publish it only on success.

    The caller must close its writer before leaving the context. With overwrite
    disabled, publication also refuses a destination created concurrently.
    """
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Let the writer create the file with its normal umask rather than inheriting
    # mkstemp's 0600 mode (outputs may be read outside the processing container).
    with tempfile.TemporaryDirectory(prefix=f".{destination.name}.", dir=destination.parent) as directory:
        temporary = Path(directory) / "output.tmp"
        yield temporary
        if overwrite:
            os.replace(temporary, destination)
        else:
            os.link(temporary, destination)
