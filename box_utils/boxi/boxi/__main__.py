#!/usr/bin/python3
# PYTHON_ARGCOMPLETE_OK
import argparse, argcomplete
import logging as log
import importlib

command_register = [
    "black",
    "build",
    "delete_data",
    "get_data",
    "initial_clock_sync",
    "kill",
    "launch",
    "launch_on_host",
    "push",
    "ptp_restart",
    "set_time_from_opc",
    "git_set_user",
    "shutdown",
    "load_calib",
    "mission_summary",
    "integration_test",
    "repair_bags",
    "set_cpt7_time",
    "convert_mcap",
]


def main():
    parser = get_parser()
    parser.set_defaults(main=lambda x: parser.print_help())
    argcomplete.autocomplete(parser)
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
