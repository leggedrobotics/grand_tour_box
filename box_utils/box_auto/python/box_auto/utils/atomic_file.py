"""Publish complete local outputs without exposing partial files to later stages."""

from contextlib import contextmanager
import ctypes
import errno
import fcntl
import os
from pathlib import Path
import tempfile


def publish_no_replace(source, destination):
    """Publish a complete output without replacing another pipeline's output.

    Linux renameat2 provides an atomic no-replace operation. Some removable
    filesystems implement neither it nor hard links. There we serialize the
    existence check and rename using flock on the destination's parent directory
    (the same protocol as box_export's C++ writer). The rename is still atomic,
    but writers outside this locking protocol must be coordinated separately.
    """
    rename = getattr(ctypes.CDLL(None, use_errno=True), "renameat2", None)
    if rename is not None:
        rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
        rename.restype = ctypes.c_int
        # AT_FDCWD = -100; RENAME_NOREPLACE = 1.
        if rename(-100, os.fsencode(source), -100, os.fsencode(destination), 1) == 0:
            return
        error = ctypes.get_errno()
        if error not in (errno.ENOSYS, errno.EINVAL, errno.EOPNOTSUPP):
            raise OSError(error, os.strerror(error), str(destination))

    # Keep the stronger publication operation on filesystems supporting links
    # even when their no-replace rename operation is unavailable.
    try:
        os.link(source, destination)
        return
    except OSError as error:
        if error.errno not in (errno.ENOSYS, errno.EINVAL, errno.EOPNOTSUPP, errno.EPERM):
            raise

    parent_fd = os.open(os.fspath(Path(destination).parent), os.O_RDONLY | os.O_DIRECTORY)
    try:
        fcntl.flock(parent_fd, fcntl.LOCK_EX)
        try:
            os.lstat(destination)  # Also refuse a dangling symlink.
        except FileNotFoundError:
            os.rename(source, destination)
        else:
            raise FileExistsError(errno.EEXIST, os.strerror(errno.EEXIST), str(destination))
    finally:
        os.close(parent_fd)  # Closing releases the directory lock, including on failure.


@contextmanager
def atomic_output(destination, overwrite=True):
    """Yield a temporary path next to destination and publish it only on success.

    The caller must close its writer before leaving the context. With overwrite
    disabled, publication refuses an existing destination. See publish_no_replace
    for the coordination required on filesystems without native no-replace support.
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
            publish_no_replace(temporary, destination)


@contextmanager
def atomic_directory(destination):
    """Publish a newly generated report directory; preserve existing reports.

    Writers own only the staging directory. Exceptions remove that staging
    directory, including a partially rendered frame sequence or plot collection.
    The filesystem coordination contract is the same as publish_no_replace.
    """
    import shutil

    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if os.path.lexists(destination):
        raise FileExistsError(destination)
    pending = Path(tempfile.mkdtemp(prefix="." + destination.name + ".", dir=destination.parent))
    try:
        yield pending
        publish_no_replace(pending, destination)
    finally:
        if pending.exists():
            shutil.rmtree(pending)
