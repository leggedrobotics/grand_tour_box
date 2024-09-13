eval $(ssh-agent)
docker build --no-cache --ssh default --progress=plain -t rslethz:noetic-pi-focal ./
