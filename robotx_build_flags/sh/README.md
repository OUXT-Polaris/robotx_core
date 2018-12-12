# crosscompile on AWS

## server setup
You have to run this setup when you run this for the first time.
`bash setup.sh`
It will install ROS, CUDA compiler, TensorRT, etc on your aws arm server

## compile
1. get `arm.sh`, `arm.pem`, `wamv.pem` from admin. place it to this `sh` directory
1. `chmod 600 arm.pem && chmod 600 wamv.pem`
1. `make update`(if you want only pull -> catkin_make) or `make build`(clone -> catkin_make)
