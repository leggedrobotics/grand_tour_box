export DOCKER_BUILDKIT=1
## Set package root path
PKGROOT="$( realpath "$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null 2>&1 && pwd )"/../ )"

docker build --progress=plain --ssh default -t "leggedrobotics/box" -f "box.Dockerfile" "$PKGROOT/"
