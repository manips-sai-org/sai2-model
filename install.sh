# script to install sai2-model

cd rbdl
mkdir build
cd build
cmake -DRBDL_BUILD_ADDON_URDFREADER=ON -DCMAKE_BUILD_TYPE=Release .. && make -j8

cd ../..
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release .. && make -j8

cd ..
