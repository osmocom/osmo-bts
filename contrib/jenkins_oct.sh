#!/usr/bin/env bash

set -ex

rm -rf deps/install
mkdir deps || true
cd deps

# Get the headers..
git clone git://git.osmocom.org/openbsc || true
cd openbsc
git pull --rebase


# Build the dependency
cd ../

osmo-deps.sh libosmocore
cd libosmocore
autoreconf --install --force
./configure --prefix=$PWD/../install
$MAKE $PARALLEL_MAKE install

cd ../
osmo-deps.sh libosmo-abis
cd libosmo-abis
autoreconf --install --force
PKG_CONFIG_PATH=$PWD/../install/lib/pkgconfig ./configure --prefix=$PWD/../install
PKG_CONFIG_PATH=$PWD/../install/lib/pkgconfig $MAKE $PARALLEL_MAKE install

cd ../
if ! test -d layer1-api;
then
  git clone git://git.osmocom.org/octphy-2g-headers layer1-api
fi

cd layer1-api
git fetch origin
if [ $FIRMWARE_VERSION = "master" ];
then
git reset --hard origin/master
else
git reset --hard $FIRMWARE_VERSION
fi


# Build osmo-bts
cd ../../
autoreconf --install --force
PKG_CONFIG_PATH=$PWD/deps/install/lib/pkgconfig ./configure --with-openbsc=$PWD/deps/openbsc/openbsc/include --with-octsdr-2g=$PWD/deps/layer1-api/ --enable-octphy
PKG_CONFIG_PATH=$PWD/deps/install/lib/pkgconfig $MAKE $PARALLEL_MAKE
PKG_CONFIG_PATH=$PWD/deps/install/lib/pkgconfig LD_LIBRARY_PATH=$PWD/deps/install/lib $MAKE check
DISTCHECK_CONFIGURE_FLAGS="--with-octsdr-2g=$PWD/deps/layer1-api/ --with-openbsc=$PWD/deps/openbsc/openbsc/include --enable-octphy" PKG_CONFIG_PATH=$PWD/deps/install/lib/pkgconfig LD_LIBRARY_PATH=$PWD/deps/install/lib $MAKE distcheck