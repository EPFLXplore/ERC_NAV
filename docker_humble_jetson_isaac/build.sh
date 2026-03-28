#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker build --progress=plain --network=host \
    -t ghcr.io/epflxplore/nav:humble-jetson-isaac \
    -f "$SCRIPT_DIR/Dockerfile" \
    "$SCRIPT_DIR/.."
