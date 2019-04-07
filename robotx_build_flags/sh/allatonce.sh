#!/bin/bash
set -Ceuo pipefail

HOST=$OUXT_AWS_HOSTNAME
KEY=$OUXT_AWS_KEYFILE
USER=$OUXT_AWS_USERNAME
echo "starting with $HOST, $KEY, $USER"

# compile and get files
ssh -i $KEY $USER@$HOST < compile.sh
scp -i $KEY $USER@$HOST:/home/$USER/catkin_ws.tar.gz .

echo 'finished'
