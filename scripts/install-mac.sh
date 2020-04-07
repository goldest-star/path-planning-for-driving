#! /bin/bash
brew install openssl libuv cmake zlib

# Download uWebSockets
wget https://github.com/uNetworking/uWebSockets/archive/v0.13.0.tar.gz
tar -xvzf v0.13.0.tar.gz
cd uWebSockets-0.13.0

# Patch the cmakefile
wget https://raw.githubusercontent.com/udacity/CarND-Extended-Kalman-Filter-Project/master/cmakepatch.txt
patch CMakeLists.txt < cmakepatch.txt
mkdir build && cd build
cmake -DOPENSSL_ROOT_DIR=/usr/local/opt/openssl -DOPENSSL_LIBRARIES=/usr/local/opt/openssl/lib ..
make
sudo make install
cd ../..
if [ ! -f "/usr/lib/libuWS.so" ]; then
    sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
fi
rm -rf v0.13.0.tar.gz uWebSockets-0.13.0

# Specific CPP dependencies
bash scripts/install-cppdeps.sh
