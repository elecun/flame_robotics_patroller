# download sdk
```
https://github.com/ouster-lidar/ouster-sdk/tags
```

# build lidar sdk on linux(ubuntu)
```
$ sudo apt install build-essential cmake libeigen3-dev libcurl4-openssl-dev \
                   libtins-dev libpcap-dev libglfw3-dev libpng-dev \
                   libflatbuffers-dev libceres-dev libtbb-dev \
                   robin-map-dev
$ mkdir build
$ cd build
$ cmake  <path to ouster-sdk/CMakeLists.txt> -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON
$ cmake --build . -- -j$(nproc)
```

-DBUILD_VIZ=OFF                    # Do not build the sample visualizer
-DBUILD_PCAP=OFF                   # Do not build pcap tools
-DBUILD_OSF=OFF                    # Do not build OSF lib
-DBUILD_EXAMPLES=ON                # Build C++ examples
-DBUILD_TESTING=ON                 # Build tests
-DBUILD_SHARED_LIBRARY=ON          # Build the shared library and binary artifacts