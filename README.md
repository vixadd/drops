# drops
DROPS - Dynamic Realtime Obstacle Pathing System
CNU UAS LAB

Prerequisites
-------------

The following (debian) packages are necessary for this project:

```
sudo apt-get install g++-4.8 g++ make libboost1.54-all-dev libssl-dev
```

Cloning
-------

To install this software, this git repo must be cloned using `git clone`. After cloning run `git submodule update --init` to initialize the SBPL submodule.

Building
--------

## SBPL

To build and install SBPL as a dynamic library run the following commands from the root of the project:

(Note: you can use the `-j 4` option on make to speed it up)

```
cd lib/sbpl
mkdir build && cd build
cmake ..
make
sudo make install
```

## C++ REST SDK

To build and install the REST SDK do the following from the root of the project:

(Note: you can use the `-j 4` option on make to speed it up)

```
cd lib/cpprestsdk/Release
mkdir build.release
cd build.release
CXX=g++-4.8 cmake .. -DCMAKE_BUILD_TYPE=Release
make
sudo make install
```

There seems to be an issue with the install permissions, so run the following to make the library executable, then add it to the linker.

```
sudo chmod +x /usr/local/lib/libcpprest.so*
sudo ldconfig
```
