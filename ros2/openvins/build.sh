#!/usr/bin/env bash
# Build the OpenVINS image (pinned commit, see Dockerfile). Takes ~10-20 min the first time.
set -euo pipefail
docker build -t isaac-exploration/openvins:jazzy "$(dirname "$0")"
