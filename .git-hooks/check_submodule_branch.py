import sys
from git import Repo
import configparser


def check_submodule_branches():
    # Read .gitmodules file
    config = configparser.ConfigParser()
    config.read(".gitmodules")

    repo = Repo(".")
    errors_found = False

    # Check each submodule
    for section in config.sections():
        path = config[section]["path"]
        expected_branch = config[section]["branch"]

        try:
            submodule = repo.submodule(path)
            current_branch = submodule.module().active_branch.name

            if current_branch != expected_branch:
                print(
                    f"Error: Submodule '{path}' is on branch '{current_branch}' "
                    f"but should be on '{expected_branch}'"
                )
                errors_found = True

        except Exception as e:
            print(f"Warning: Could not check submodule '{path}': {str(e)}")
            errors_found = True

    return 1 if errors_found else 0


if __name__ == "__main__":
    sys.exit(check_submodule_branches())
