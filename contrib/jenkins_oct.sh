#!/bin/sh

# shellcheck source=contrib/jenkins_common.sh
. $(dirname "$0")/jenkins_common.sh

osmo-build-dep.sh libosmocore

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"

osmo-build-dep.sh libosmo-abis

cd "$deps"
osmo-layer1-headers.sh oct "$FIRMWARE_VERSION"
cd "$base"

set +x
echo
echo
echo
echo " =============================== osmo-bts-octphy ==============================="
echo
set -x

autoreconf --install --force
./configure --with-openbsc="$deps/openbsc/openbsc/include" --with-octsdr-2g="$deps/layer1-headers/" --enable-octphy
$MAKE $PARALLEL_MAKE
$MAKE check \
  || cat-testlogs.sh
DISTCHECK_CONFIGURE_FLAGS="--with-octsdr-2g=$deps/layer1-headers/ --with-openbsc=$deps/openbsc/openbsc/include --enable-octphy" \
  $MAKE distcheck \
  || cat-testlogs.sh
