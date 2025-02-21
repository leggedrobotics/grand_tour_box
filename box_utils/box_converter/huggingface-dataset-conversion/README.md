# Running the Conversion

Navigate to the folder that contains this file.

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
docker build -t grand-tour-dataset . && docker run -v "$(pwd):/app" --rm -it grand-tour-dataset
```

## Run the Dataset Builder

Once you are inside the docker container you can run the dataset builder:

```bash
python -m dataset_builder
```
