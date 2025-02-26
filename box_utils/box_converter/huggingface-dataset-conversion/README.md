# Running the Conversion

Navigate to the folder that contains this file.

## Setting up the Local Environment

we set this up to facilitate the downloading from kleinkram and uploading to huggingface

```bash
virtualenv -ppython3.12 .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Downloading the Mission Data

Download the required `.mcap` files for the mission you want to conver and place them into the `data/files` folder.
I.e., if you are using the kleinkram cli you can do:

```bash
klein download -p GrandTour -m ... --dest data/files "*.mcap"
```

This will get changed later on, but thats how its done right now.

## Starting the Docker Container

Run the following command to start the docker container:

```bash
docker build --ssh default -t grand-tour-dataset . && docker run -v "$(pwd):/app" --rm -it grand-tour-dataset
```

## Run the Dataset Builder

Once you are inside the docker container you can run the dataset builder:

```bash
python -m dataset_builder
```

## Configuring the Converter

Currently the converter can be configured by editing the `config/full.yaml` file.

The config has two parts, the `metadata` and the `data` part.
The first is used to specify the parsing and conversion of the metadata.
I.e. topics or parts of topics that remain largly static.
The latter is used to specify the parsing and conversion of the "data", which is anything that is not metadata.

#### `metadata` section

#### `data` section

The `data` section supports the following keys:

- `image_topics`: Specifies all topics that should be parsed using the image parser.
  It takes a list of the following entries:

  ```yaml
  - alias: *desired topic name*
    file: *mcap file name*
    topic: *topic inside the mcap file*
    compressed: *true or false, tells the parser if msg is CompressedImage or Image*
    format: *png or jpg, desired format to save the images*
  ```

- `lidar_topics`: Specifies all topics that should be parsed as a point cloud.
  It takes a list of the following entries:

  ```yaml
  - alias: *desired topic name*
    file: *mcap file name*
    topic: *topic inside the mcap file*
    max_points: *max number of point cloud points that can be contained in a message*
    attributes:
      - ... *list of attributes that are contained in each point*
  ```

- `pose_topics`: Specifies all topics that should be parsed as a pose.
  It takes a list of the following entries:

  ```yaml
  - alias: *desired topic name*
    file: *mcap file name*
    topic: *topic inside the mcap file*
    covariance: *true or false, tells the parser if the pose has a covariance matrix*
  ```

- `imu_topics`, `odometry_topics`, `nav_sat_fix_topics`, `point_topics`, `singleton_transform_topics`:
  Specify all topics that should be parsed with their respective parsers.
  They all take a list of the following entries:

  ```yaml
  - alias: *desired topic name*
    file: *mcap file name*
    topic: *topic inside the mcap file*
  ```

  Note that the `singleton_transform_topics` are generally very similar to `pose_topics`, i.e. they are a list containing a single pose.

## Profiling the Converter

```bash
python -m cProfile -o o.pstat dataset_builder.py
gprof2dot -f pstats o.pstat | dot -Tsvg -o o.svg
```

##### Profiling Results

- 60-70% of the time is spent reading and writing images
- 20% of time is spent inside the mcap message iterator function
