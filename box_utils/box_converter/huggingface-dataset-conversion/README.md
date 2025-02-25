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

Currently the converter can be configured by editing the `config/default.yaml` file.

The config has two parts, the `metadata` and the `data` part.
The first is used to specify the parsing and conversion of the metadata.
I.e. topics or parts of topics that remain largly static.
The latter is used to specify the parsing and conversion of the "data", which is anything that is not metadata.

```yaml
```

## Profiling the Converter

```bash
python -m cProfile -o o.pstat dataset_builder.py
gprof2dot -f pstats o.pstat | dot -Tsvg -o o.svg
```
