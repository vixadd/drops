# DROPS
DROPS - Dynamic Realtime Obstacle Pathing System
CNU UAS LAB

Prerequisites
-------------

The following (debian) packages are necessary for this project:

##Linux

```
sudo apt-get install g++-4.8 g++ make libboost1.54-all-dev libssl-dev astyle
```

##OSX

```
brew install openssl astyle boost --c++11
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

Debug
-----

## DROPS
To do a debug build (defines the DEBUG precompiler symbol and adds symbols for gdb), run `make debug`.

## SBPL

If debugging it might be helpful to set SBPL into debug mode. Sadly it's not as easy for SBPL to be put into debug mode.

To do so, change the `#define DEBUG 0` line in `lib/sbpl/src/include/sbpl/config.h` to `#define DEBUG 1` then repeat the build steps listed above.

Directories
------

Here is a discription for some of the directory structure:

 * lib - Libraries used in drops (sbpl and cpprestsdk)
 * bin - binaries created by make
 * obj - intermediate build files
 * src - the source files
 * res - resources used by the project

Motion Primatives
-----------------

Motion primatives are essential to the planning. The planner uses motion primatives to know the possible moves the vehicle can take and their relative costs. To generate motion primatives a Matlab script (*.m) is provided in the `res` folder.

On Linux the Matlab script can be run using Octave. Octave is FOSS alternative to Matlab. On Debian systems it can be installe using `sudo apt-get install octave`.

To generate the motion primative:

 * open ocatve: `octave`
 * load the matlab file: `load genprim_plane.m`
 * execute the function: `genprim_plane("output_filename.mprim")`

This will create a motion primative file called `output_filename.mprim` with the motion primatives.
