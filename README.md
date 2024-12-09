# Sai-Model

This is SAI robot model library for robot kinematics and dynamics.
It uses [RBDL](https://rbdl.bitbucket.io/) and adds function to facilitate the implementation of the whole body control framework from Stanford Robotics Lab.

## Dependencies
sai-model depends on eigen3 and sai-urdfreader.
You can get sai-urdfreader [there](https://github.com/manips-sai-org/sai-urdfreader).

To install eigen3 on Ubuntu :
```
sudo apt-get install libeigen3-dev
```
on Mac :
```
brew install eigen
```

## Build instructions 
You can use the provided install script for automatic install
```
sh install.sh
```
Or you can install manually :
 * First go to the rbdl folder and make rbdl
 ```
cd rbdl
mkdir build
cd build
cmake .. && make -j8
cd ../..
```
 * Then you can make sai-model from the base directory
```
mkdir build
cd build
cmake .. && make -j8
```

## Run the examples
Go to the build/examples/one_of_the_examples folder and run the example. For example 1 :
```
cd build/examples/01-create_model_from_file
./01-create_model_from_file
```

## Documentation
The documentation can also be accessed online at the following [link](https://manips-sai-org.github.io/sai-model/)

It can also be generated locally with doxygen:
```
cd docs
doxygen
```

## License
Currently pending licensing. PLEASE DO NOT DISTRIBUTE.

## For questions, contact:
mjorda@stanford.edu or mjorda@jorda-tech.com