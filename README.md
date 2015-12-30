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

To build and install SBPL as a dynamic library run the following commands

```
cd lib/sbpl
mkdir build && cd build
cmake ..
make
sudo make install
```
