import argparse
import argcomplete
import logging as log
import importlib

command_register = [
    "push",
    "black",
    "start",
    "initial_clock_sync",
    "delete_data",
    "set_time_to_opc",
    "restart_ptp",
    "stop_all",
    "start_all",
    "get_data",
    "kill_all",
]


def main():
    parser = get_parser()
    parser.set_defaults(main=lambda x: parser.print_help())
    try:
        argcomplete.autocomplete(parser)
    except NameError:
        pass
    args = parser.parse_args()
    args.main(args)


def get_parser():
    parser = argparse.ArgumentParser(description="Boxi")

    subparsers = parser.add_subparsers()
    for cmd_name in command_register:
        try:
            cmd_module = importlib.import_module("." + cmd_name.replace("-", "_"), "boxi")
        except ModuleNotFoundError as e:
            log.error(f"Failed to load a command implementation for '{cmd_name}': {e}")
            continue
        cmd_parser = subparsers.add_parser(cmd_name)
        cmd_module.add_arguments(cmd_parser)
    return parser


if __name__ == "__main__":
    main()
