#!/bin/bash
ffmpeg -ss 0.5 -video_size 320x240 -framerate 30.000030 -f avfoundation -i "0" -t 1 capture.jpg
