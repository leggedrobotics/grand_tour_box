from boxi import BOX_ROOT_DIR, shell_run
import argparse
import configparser
import os
import subprocess
import sys
from pathlib import Path
from time import sleep


def add_arguments(parser):
    yamls = [s.stem for s in Path(BOX_ROOT_DIR).joinpath("box_launch/tmux").rglob("*.yaml")]
    parser.set_defaults(main=main)
    parser.add_argument("-c", choices=yamls, help="tmux configuration file name", default="box_opc")
    return parser

# Use a .gitignored .ini file to store ssid and password. Example format:
# [WiFi]
# ssid=wifiNetworkName
# password=wifiPassword
def load_wifi_config(config_file='wifi_config.ini'):
    config_file_path = os.path.expanduser('~/'+config_file)
    if not os.path.exists(config_file_path):
        print("Place a wifi_config.ini file in your user home directory.")
        sys.exit(1)
    config = configparser.ConfigParser()
    config.read(config_file_path)
    return config['WiFi']['ssid'], config['WiFi']['password']

def connect_to_wifi(ssid, password):
    print(f"Attempting to connect to Grand Tour Box router.")
    try:
        subprocess.check_call(["nmcli", "dev", "wifi", "connect", ssid, "password", password])
        print("Successfully connected")
    except subprocess.CalledProcessError as e:
        print(f"Failed to connect. Error: {e}")

def main(args):
    ssid, password = load_wifi_config()
    connect_to_wifi(ssid, password)
    cfg = os.path.join(BOX_ROOT_DIR, "box_launch/tmux", args.c + ".yaml")
    shell_run(f"tmuxp load  {cfg} -d")
