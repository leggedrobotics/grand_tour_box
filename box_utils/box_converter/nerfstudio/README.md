# Convert docker to tar
```
/home/jonfrey/git/grand_tour_box/box_utils/box_auto/docker/build.sh --kleinkram --no-cache; cd /data/GrandTour/docker; apptainer build --sandbox grand-tour-kleinkram.sif docker-daemon://rslethz/grand_tour_box:kleinkram; sudo tar -cvf grand-tour-kleinkram.tar grand-tour-kleinkram.sif; scp grand-tour-kleinkram.tar jonfrey@euler.ethz.ch:/cluster/work/rsl/jonfrey/grand_tour/containers


```

Sync local repo to euler:
/home/jonfrey/git/grand_tour_box/box_utils/box_converter/nerfstudio/sync_to_cluster.sh

Debug
```bash
srun --account=es_hutter --ntasks=1 --cpus-per-task=4 --gpus=1 --time=4:00:00 --mem-per-cpu=8024 --tmp=50000 --pty bash

# Copy container
tar -xf /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-kleinkram.tar  -C $TMPDIR

# Run test script 
apptainer exec --nv --writable --bind /cluster/scratch/jonfrey:/out --bind /cluster/home/jonfrey/grand_tour_box:/home/catkin_ws/src --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" --containall $TMPDIR/grand-tour-dataset.sif /bin/bash -c 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH; export HOME=/home; /entrypoint_euler.sh  python /home/catkin_ws/src/grand_tour_box/box_utils/box_converter/nerfstudio/nerf_studio.py --mission_uuid d7525079-5564-461e-a144-e7479247d268'

# WARNING: Cluster requires different endpoint then local pc. Therefore copying over ~/.kleinkram.json wont work.
#          Instead run container and login using kleinkram login via CLI and copy manual.
```


$ TODO make the kleinkram image build successfully
$ Add entrypoint for euler
$ Mount for development on euler the grand_tour_box repo into the singularity one
ç Check if everything isg ood


# Plan having two docker files
- Generating image data
- Generating wavemaps
- Generating depth images
- Training the gaussian splatting model


- build the container
    ```
    DOCKER_BUILDKIT=1 docker compose build
    ```

- run the container
    ```
    docker compose up -d && docker exec -it nerfstudio bash
    ```