#! /bin/bash

ARCH=$(uname -m)

if [ "$ARCH" == "x86_64" ]; then
    DOCKERFILE="wheels/Dockerfile.x86_64"
elif [ "$ARCH" == "arm64" ] || [ "$ARCH" == "aarch64" ]; then
    DOCKERFILE="wheels/Dockerfile.aarch64"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi
set -e
root_dir=$(dirname $(dirname $(realpath $0)))

cd $root_dir
docker build -t arx5-sdk --file $DOCKERFILE .
docker create --name arx5-sdk-container arx5-sdk
docker cp arx5-sdk-container:/root/arx5-sdk/wheelhouse wheels/
docker rm arx5-sdk-container

