"""Own a private ROS master without changing another pipeline's ROS session."""

import os
import signal
import socket
import subprocess
import tempfile
import time
import xmlrpc.client
from contextlib import contextmanager


@contextmanager
def private_ros_session():
    # ROS logging creates symlinks. Keep control files on the host filesystem,
    # even when TMPDIR points at a removable dataset drive without symlinks.
    with tempfile.TemporaryDirectory(prefix="box_auto_ros_", dir="/tmp") as directory:
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        env = dict(
            os.environ,
            ROS_MASTER_URI="http://127.0.0.1:%d" % port,
            ROS_HOSTNAME="127.0.0.1",
            ROS_HOME=directory,
            ROS_LOG_DIR=os.path.join(directory, "log"),
        )
        env.pop("ROS_IP", None)
        process = subprocess.Popen(
            ["roscore", "-p", str(port)],
            env=env,
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        try:
            server = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"])
            for _ in range(100):
                if process.poll() is not None:
                    raise RuntimeError("Private ROS master exited during startup")
                try:
                    if server.getPid("/box_auto")[0] == 1:
                        break
                except OSError:
                    pass
                time.sleep(0.1)
            else:
                raise RuntimeError("Private ROS master failed to start")
            yield env
        finally:
            # The process group contains only the master launched above.
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
