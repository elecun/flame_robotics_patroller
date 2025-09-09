# install pre-requisites

* 

1. download & install Kvaser Linux Driver and SDK 5.49.x
* kvaser.com/download
```
$ sudo apt-get install build-essential pkg-config dkms
$ sudo apt-get install linux-headers-`uname -r`
$ make
$ sudo make install
$ sudo make load
```