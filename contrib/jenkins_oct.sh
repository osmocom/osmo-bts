#!/usr/bin/env bash

set -ex

base="$PWD"
deps="$base/deps"
inst="$deps/install"
export deps inst

mkdir "$deps" || true
rm -rf "$inst"

# Get the headers..
cd "$deps"
git clone git://git.osmocom.org/openbsc || true
cd openbsc
git pull --rebase
cd "$base"

osmo-build-dep.sh libosmocore

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"

osmo-build-dep.sh libosmo-abis

cd "$deps"
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

cd "$base"

set +x
echo
echo
echo
echo " =============================== osmo-bts-octphy ==============================="
echo
set -x

autoreconf --install --force
./configure --with-openbsc="$deps/openbsc/openbsc/include" --with-octsdr-2g="$deps/deps/layer1-api/" --enable-octphy
$MAKE $PARALLEL_MAKE
$MAKE check \
  || cat-testlogs.sh
DISTCHECK_CONFIGURE_FLAGS="--with-octsdr-2g=$deps/layer1-api/ --with-openbsc=$deps/openbsc/openbsc/include --enable-octphy" \
  $MAKE distcheck \
  || cat-testlogs.sh
