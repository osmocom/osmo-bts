#!/bin/sh

# shellcheck source=contrib/jenkins_common.sh
. $(dirname "$0")/jenkins_common.sh

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"

osmo-build-dep.sh libosmocore

osmo-build-dep.sh libosmo-abis

cd "$deps"

# Get osmo-pcu for pcuif_proto.h
osmo-deps.sh osmo-pcu

osmo-layer1-headers.sh oct "$FIRMWARE_VERSION"

cd "$base"

set +x
echo
echo
echo
echo " =============================== osmo-bts-octphy+trx ==============================="
echo
set -x

autoreconf --install --force
configure_flags="\
  --with-openbsc=$deps/openbsc/openbsc/include \
  --with-osmo-pcu=$deps/osmo-pcu/include \
  --with-octsdr-2g=$deps/layer1-headers/ \
  --enable-octphy \
  --enable-trx \
  "
./configure $configure_flags
$MAKE $PARALLEL_MAKE
$MAKE check \
  || cat-testlogs.sh
DISTCHECK_CONFIGURE_FLAGS="$configure_flags" \
  $MAKE distcheck \
  || cat-testlogs.sh
