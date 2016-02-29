#!/bin/bash

echo "arguments fps output_file image_type"

# mencoder mf://*.$3 -mf fps=$1:type=$3 -ovc x264 -x264encopts crf=20 -oac copy -o $2

mencoder mf://*.$3 -mf fps=$1:type=$3 -ovc lavc -lavcopts vcodec=wmv2 -oac copy -o $2

#mencoder mf://*.$3 -mf fps=$1:type=$3 -ovc x264 -x264encopts bitrate=800:pass=1 -oac copy -o $2
#mencoder mf://*.$3 -mf fps=$1:type=$3 -ovc x264 -x264encopts bitrate=800:pass=2 -oac copy -o $2




