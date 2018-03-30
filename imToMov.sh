#!/bin/bash

echo "arguments fps output_file image_type"

ls -v --format=single-column *.$3 > list.txt

mencoder mf://@list.txt -mf fps=$1:type=$3 -ovc x264 -x264encopts crf=20 -oac copy -o $2
# mencoder mf://@list.txt -mf fps=$1:type=$3 -ovc lavc -lavcopts vcodec=wmv2 -oac copy -o $2
# mencoder mf://@list.txt -mf fps=$1:type=$3 -ovc x264 -x264encopts bitrate=800:pass=1 -oac copy -o $2
# mencoder mf://@list.txt -mf fps=$1:type=$3 -ovc x264 -x264encopts bitrate=800:pass=2 -oac copy -o $2

rm list.txt
