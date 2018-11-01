# robotx_core packages
core packages in [robotx_packages](https://github.com/OUXT-Polaris/robotx_packages).  
robotx_core packages are ROS packages for Jetson TX2 or other embeded boards which is on our wam-v.

| *master* | *develop* |
|----------|-----------|
|[![Build Status](https://travis-ci.org/OUXT-Polaris/robotx_core.svg?branch=master)](https://travis-ci.org/OUXT-Polaris/robotx_core)|[![Build Status](https://travis-ci.org/OUXT-Polaris/robotx_core.svg?branch=develop)](https://travis-ci.org/OUXT-Polaris/robotx_core)|

# How to cross-compile
1.update bloom  
See also https://qiita.com/musubi05/items/8d36f96122ef31145915.  
```
git clone https://github.com/OUXT-Polaris/bloom
cd bloom
sudo python setup.py install

cd <robotx_core dir>
sudo apt update
sudo apt install fakeroot dpkg-dev debhelper
source build_dpkg.bash
``` 

# Model conversion is required before first run of robotx_recognition
Before running, converted model weight file of CNN detector (robotx_recognition/data/robotx.plan) should be downloaded/generated.
To generate the model, run the snippet below on the machine (with TensorRT installed).
```
bash robotx_recognition/setup_model.sh
```
