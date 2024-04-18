import copy
import os
import shlex
import subprocess
import time

def shell_run(cmd, cwd=None, env={}, time=True, continue_on_error=True):
    """Execute shell command."""
    # Support both list and str format for commands
    # if isinstance(cmd, str):
    #     cmd = shlex.split(cmd)

    # Set up environmental variables
    env_variables = copy.deepcopy(os.environ)
    env_variables.update(env)

    # Execute command
    try:
        p = subprocess.Popen(cmd, cwd=cwd, env=env_variables, shell=True)
    except Exception as e:
        raise RuntimeError(f"{e} --- while executing {cmd}")

    if p.wait() != 0:
        print()
        if not continue_on_error:
            raise RuntimeError(f"Error Return non 0 --- while executing {cmd}")


if __name__ == "__main__":
    shell_run("ls $HOME")
