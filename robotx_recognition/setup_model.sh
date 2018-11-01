#!/bin/bash
set -Ceu
cd $(dirname $0)

echo "[1/3] download uff to data/robotx.uff"
wget https://s3-us-west-2.amazonaws.com/cnn-prediction-weights/resnet_v1_50_ft_double_longer_1022.uff -O data/robotx.uff

echo "[2/3] compile converter"
cd model_convert
mkdir -p build
cd build
cmake ..
make

echo "[3/3] convert"
./plan ../../data/robotx.uff ../../data/robotx.plan images resnet_v1_50/SpatialSqueeze
