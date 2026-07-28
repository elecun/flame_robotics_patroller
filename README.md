# flame_robotics_patroller

# environments
- > ubuntu 22.04.5
- > python 3.10.12

## Setup on Ubuntu (22.04.5)
1. install dependent packagaes
```
$ sudo apt-get install libzmq3-dev libopencv-dev
```

## Kvaser CAN Install
```
$sudo apt-get install linux-headers-`uname -r` 
$sudo apt-get install pkg-config 
$wget --content-disposition "https://www.kvaser.com/downloads-kvaser/?utm_source=software&utm_ean=7330130980754&utm_status=latest"
$tar xvzf linuxcan.tar.gz
$cd linuxcan
$make -j
$sudo make install
```