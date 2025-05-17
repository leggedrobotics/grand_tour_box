
## 1. extracting NerfStudio dataset from rosbags on euler (setup)
Setup correct .kleinkram.json on euler - for this ensure the correct endpoint setting:
```shell
klein endpoint prod_no_proxy https://api.datasets.leggedrobotics.com http://minio.datasets.leggedrobotics.com:45322
```

Sync local repo to euler:
```
/home/jonfrey/git/grand_tour_box/box_utils/box_converter/nerfstudio/sync_to_cluster.sh
```

Obtaining an interactive session:
```
srun --account=es_hutter --ntasks=1 --cpus-per-task=4 --gpus=1 --time=4:00:00 --mem-per-cpu=8024 --tmp=50000 --pty bash
```

Copy container to local SSD:
```
tar -xf /cluster/work/rsl/jonfrey/grand_tour/containers/grand-tour-kleinkram.tar  -C $TMPDIR; mkdir $TMPDIR/grand-tour-kleinkram.sif/out
```

Command to run test mission:
```
apptainer exec --nv --writable --bind /cluster/scratch/jonfrey:/out --bind /cluster/home/jonfrey/grand_tour_box:/home/catkin_ws/src/grand_tour_box --env KLEINKRAM_CONFIG="$(cat ~/.kleinkram.json)" --containall $TMPDIR/grand-tour-kleinkram.sif /bin/bash -c 'export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH; source /home/opencv_gtsam_ws/devel/setup.bash; source /home/catkin_ws/devel/setup.bash; export HOME=/home; export KLEINKRAM_ACTIVE=ACTIVE; /entrypoint_euler.sh  python3 /home/catkin_ws/src/grand_tour_box/box_utils/box_converter/nerfstudio/nerf_studio.py --mission_uuid 4ae7f9bc-ee51-4ba7-97c7-6a86fe765ab8 --head 10'
```

Script to update docker container on euler:
```
/home/jonfrey/git/grand_tour_box/box_utils/box_auto/docker/build.sh --kleinkram --no-cache; cd /data/GrandTour/docker; apptainer build --sandbox grand-tour-kleinkram.sif docker-daemon://rslethz/grand_tour_box:kleinkram; sudo tar -cvf grand-tour-kleinkram.tar grand-tour-kleinkram.sif; scp grand-tour-kleinkram.tar jonfrey@euler.ethz.ch:/cluster/work/rsl/jonfrey/grand_tour/containers
```

Schedule all jobs on euler:
```
python3 /home/jonfrey/git/grand_tour_box/box_utils/box_converter/nerfstudio/create_submit_files.py
```


## 2. Run NeRFStudio
