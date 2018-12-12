# crosscompile on AWS

## server setup
You have to run this setup when you run this for the first time.
`bash setup.sh`
It will install ROS, CUDA compiler, TensorRT, etc on your aws arm server

## compile
1. get arm.sh and arm.key, then `. arm.sh` (this will set private AWS server address)
2. `bash allatonce.sh` to compile
3. you will see catkin_ws.tar.gz downloaded on this directory
