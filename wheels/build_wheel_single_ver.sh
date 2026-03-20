#! /bin/bash


PYTHON_VERSION="cp310-cp310"
# All available versions: cp38-cp38 cp39-cp39 cp310-cp310 cp311-cp311 cp312-cp312 cp313-cp313 cp314-cp314

ARCH=$(uname -m)

if [ "$ARCH" == "x86_64" ]; then
    DOCKERFILE="wheels/Dockerfile.single_ver_x86_64"
elif [ "$ARCH" == "arm64" ] || [ "$ARCH" == "aarch64" ]; then
    DOCKERFILE="wheels/Dockerfile.single_ver_aarch64"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi
set -e
root_dir=$(dirname $(dirname $(realpath $0)))

cd $root_dir
docker build -t arx5-sdk-single-ver --file $DOCKERFILE --build-arg PYTHON_VERSION="${PYTHON_VERSION}" .
docker create --name arx5-sdk-single-ver-container arx5-sdk-single-ver
docker cp arx5-sdk-single-ver-container:/root/arx5-sdk/wheelhouse wheels/
docker rm arx5-sdk-single-ver-container