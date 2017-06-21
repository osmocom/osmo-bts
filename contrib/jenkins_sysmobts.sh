#!/bin/sh

# shellcheck source=contrib/jenkins_common.sh
. $(dirname "$0")/jenkins_common.sh

osmo-build-dep.sh libosmocore

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"

osmo-build-dep.sh libosmo-abis

cd "$deps"
osmo-layer1-headers.sh sysmo "$FIRMWARE_VERSION"
mkdir -p "$inst/include/sysmocom/femtobts"
ln -s $deps/layer1-headers/include/* "$inst/include/sysmocom/femtobts/"
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
