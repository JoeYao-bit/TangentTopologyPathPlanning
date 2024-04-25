# TangentTopologyPathPlanning

Classic conventional topologically distinctive path planning relies on indicators related to the number of isolated obstacles to determine whether two paths belong to the same topology. This reliance often hinders real-time processing (< 1s) in maps with numerous isolated obstacles. 

Furthermore, most existing topologically distinctive path planning methods require multiple graph searches to obtain multiple paths, resulting in non-real-time performance when searching for hundreds of topologically distinctive paths. 

In light of these challenges, we propose an efficient path planning approach based on tangent graphs to discover multiple topologically distinctive paths. Our method diverges from existing algorithms by eliminating the need to distinguish whether two paths belong to the same topology. Instead, it generates multiple topologically distinctive paths based on the locally shortest property of tangents.

[toc]

# Download code
```
git clone git@github.com:JoeYao-bit/TangentTopologyPathPlanning.git
cd TangentTopologyPathPlanning
git submodule update --init  --recursive
mkdir build
cd build
cmake ..
make
```


# Install Dependencies

## Git
```
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libgtest-dev
```

## Boost
```
sudo apt-get install libboost-graph-dev
sudo apt-get install libboost-thread-dev
sudo apt-get install libboost-filesystem-dev
sudo apt-get install libboost-system-dev
sudo apt-get install libboost-program-options-dev
```

## Visualization tools

### OpenCV
```
sudo apt-get install libopencv-dev
```

### OpenGL
```
sudo apt-get install libglfw3-dev
```

### Pangolin 
Down load source code from https://github.com/stevenlovegrove/Pangolin
And unzip it.
```
sudo apt-get install libglew-dev
sudo apt-get install libboost-dev libboost-thread-dev libboost-filesystem-dev
cd Pangolin
mkdir build
cd build
cmake ..
make -j2
sudo make install
```

### Qt5
```
sudo apt-get install build-essential
sudo apt-get install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools
sudo apt-get install qtcreator
sudo apt-get install qt5*
```

## Calculation tools

### Eigen
```
sudo apt-get install libeigen3-dev

### SuiteSparse
sudo apt-get install libsuitesparse-dev
```

### G2O
```
sudo apt-get install qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3
mkdir build
cd build
cmake ..
make
sudo make install
```

### CGAL
```
sudo apt-get install libcgal-dev
```

### Octomap
```
git clone https://github.com/OctoMap/octomap
cd octomap/octomap
mkdir build
cd build
cmake ..
make
sudo make install
```

##  Auxiliary tools

### Gtest
```
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libgtest-dev
```

### Glog
```
git clone https://github.com/google/glog
cd glog
mkdir build && cd build
cmake -DGFLAGS_NAMESPACE=google -DCMAKE_CXX_FLAGS=-fPIC -DBUILD_SHARED_LIBS=ON ..
make
sudo make install
```

### Python
```
sudo apt install python3-pip
pip install matplotlib
```

### yaml-cpp
```
tar zxvf yaml-cpp-0.5.1.tar.gz
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=ON ..
make
sudo make install
sudo ldconfig
```

### libxml2
```
sudo apt-get install libxml2-dev
```

### argparse
```
https://github.com/jarro2783/cxxopts
```

# Usage

## Single test

```
./test_2D_path_planning
```
left click set start and right click set target, then path planning

## Massive test
```
./test_massive_comparison
```

# License
This software is released under MIT License. See LICENSE for further details.
Third party codes remain their own licenses.
