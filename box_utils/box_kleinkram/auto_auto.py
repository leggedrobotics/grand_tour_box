import os
import subprocess
from threading import Thread


def run_command(command, ssh_host=None):
    """Run a shell command locally or via SSH."""
    if ssh_host:
        command = f'ssh {ssh_host} "{command}"'
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    if process.returncode != 0:
        print(f"Error executing command: {command}\n{stderr.decode()}")
    else:
        print(stdout.decode())


def sequence_1(output_directory, all_missions):
    command1 = f"cd {output_directory}; boxi get_data --lpc --npc --directory {all_missions}"
    run_command(command1)
    command2 = f"python3 /home/rsl/git/grand_tour_box/box_utils/box_kleinkram/auto.py --data_folder {output_directory} --mission_names {all_missions} --local_hostname=opc --mode process --merge_on_opc True"
    run_command(command2)


def sequence_2(output_directory, all_missions):
    command1 = f"python3 /home/rsl/git/grand_tour_box/box_utils/box_kleinkram/auto.py --data_folder /data --mission_names {all_missions} --mode process"
    run_command(command1, ssh_host="nuc")
    command2 = f"cd {output_directory}; boxi get_data --nuc --post-only --directory {all_missions}"
    run_command(command2)


def sequence_3(output_directory, all_missions):
    command1 = f"python3 /home/rsl/git/grand_tour_box/box_utils/box_kleinkram/auto.py --data_folder /data --mission_names {all_missions} --mode process"
    run_command(command1, ssh_host="jetson")
    command2 = f"cd {output_directory}; boxi get_data --jetson --post-only --directory {all_missions}"
    run_command(command2)


def main():
    missions = "2024-12-09-09-34-43  2024-12-09-09-41-46  2024-12-09-11-28-28  2024-12-09-11-53-11"
    output_directory = "/media/rsl/Data/deployment_day_18"

    # Ensure the output directory exists
    os.makedirs(output_directory, exist_ok=True)

    # Start the sequences in parallel threads
    threads = [
        Thread(target=sequence_1, args=(output_directory, missions)),
        Thread(target=sequence_2, args=(output_directory, missions)),
        Thread(target=sequence_3, args=(output_directory, missions)),
    ]

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()


if __name__ == "__main__":
    main()
