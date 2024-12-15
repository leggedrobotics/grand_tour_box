import sys
import subprocess
import configparser


def run_git_command(command, cwd=None):
    """Run a git command and return the output."""
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            text=True,
            capture_output=True,
            check=True,
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Command '{' '.join(command)}' failed: {e.stderr.strip()}")


def check_submodule_branches():
    # Read .gitmodules file
    config = configparser.ConfigParser()
    config.read(".gitmodules")
    errors_found = False

    for section in config.sections():
        print(config)
        path = config[section]["path"]
        expected_branch = config[section].get("branch", None)

        try:
            # Run git command to get the current branch of the submodule
            actual_branch = run_git_command(["git", "branch", "-r", "--contains", "HEAD"], cwd=path)
            print("res", actual_branch)
            # Compare branches
            if actual_branch.find(expected_branch) == -1:
                print(
                    f"Error: Submodule '{path}' is on branch '{actual_branch}' " f"but should be on '{expected_branch}'"
                )
                errors_found = True

        except RuntimeError as e:
            print(f"Warning: Could not check submodule '{path}': {str(e)}")
            errors_found = True

    return 1 if errors_found else 0


if __name__ == "__main__":
    res = check_submodule_branches()
    print(res)
    sys.exit(res)
