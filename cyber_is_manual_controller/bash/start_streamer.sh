#!/bin/bash

cd "$(dirname "$0")"/../mjpg-streamer/mjpg-streamer-experimental
./mjpg_streamer -i "./input_uvc.so -d /dev/video0 -r 640x480 -f 30" -o "./output_http.so -p 8080 -w ./www"
