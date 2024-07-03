import yaml
import docker
import os


# Step 2: Build Docker images
def build_containers(build_config):
    for dockerfile, args in build_config.items():
        _ = args.get("build_args", {})
        dockerfile_path = f"docker/{dockerfile}.Dockerfile"
        image_name = f"leggedrobotics/{dockerfile}"

        try:
            home = os.path.expanduser("~")
            client.images.build(
                path=os.path.join(home, "git/grand_tour_box/box_utils/box_auto"),
                dockerfile=dockerfile_path,
                tag=image_name,
            )
            print(f"Successfully built {image_name}")
        except docker.errors.BuildError as build_error:
            print(f"Error building {image_name}: {build_error}")
        except docker.errors.APIError as api_error:
            print(f"API error building {image_name}: {api_error}")


# Step 3: Run Docker containers
def run_containers(run_config):
    for job_name, args in run_config.items():
        run_args = args.get("run_args", [])
        command_args = args.get("command_args")
        image_name = "leggedrobotics/" + args["container"]

        # Handling the volume arguments from run_args
        volumes = []
        for arg in run_args:
            if "-V " in arg:
                volumes.append(arg[3:])

        # Joining entrypoint_args to form the command
        if command_args is not None:
            command = " ".join(command_args)
        else:
            command = None

        print(f"Running container {image_name} with volumes {volumes} and command '{command}'")
        try:
            _ = client.containers.run(image=image_name, command=command, volumes=volumes, detach=False)
        except docker.errors.ContainerError as container_error:
            print(f"Error running container {image_name}: {container_error}")
        except docker.errors.ImageNotFound as image_not_found_error:
            print(f"Image not found {image_name}: {image_not_found_error}")
        except docker.errors.APIError as api_error:
            print(f"API error running container {image_name}: {api_error}")


if __name__ == "__main__":
    # Step 1: Read the YAML file
    with open("job_cfg.yaml", "r") as file:
        config = yaml.safe_load(file)

    client = docker.from_env()

    build_config = config.get("build", {})
    run_config = config.get("run", {})

    build_containers(build_config)
    run_containers(run_config)
