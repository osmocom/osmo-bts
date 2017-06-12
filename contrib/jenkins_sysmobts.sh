#!/bin/sh

# shellcheck source=contrib/jenkins_common.sh
. $(dirname "$0")/jenkins_common.sh

osmo-build-dep.sh libosmocore

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"

osmo-build-dep.sh libosmo-abis

cd "$deps"
if ! test -d layer1-api;
then
  git clone git://git.sysmocom.de/sysmo-bts/layer1-api.git layer1-api
fi

cd layer1-api
git fetch origin
if [ $FIRMWARE_VERSION = "master" ];
then
git reset --hard origin/master
else
git reset --hard $FIRMWARE_VERSION
fi
mkdir -p "$inst/include/sysmocom/femtobts"
cp include/*.h "$inst/include/sysmocom/femtobts/"

cd "$base"

set +x
echo
echo
echo
echo " =============================== osmo-bts-sysmo ==============================="
echo
set -x

autoreconf --install --force
./configure --enable-sysmocom-bts --with-openbsc="$deps/openbsc/openbsc/include"
$MAKE $PARALLEL_MAKE
$MAKE check \
  || cat-testlogs.sh
DISTCHECK_CONFIGURE_FLAGS="--enable-sysmocom-bts --with-openbsc=$deps/openbsc/openbsc/include" \
  $MAKE distcheck \
  || cat-testlogs.sh

# This will not work for the femtobts
if [ $FIRMWARE_VERSION != "femtobts_v2.7" ]; then
  $MAKE -C contrib/sysmobts-calib
fi
