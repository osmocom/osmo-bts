#!/usr/bin/env bash

set -ex

base="$PWD"
deps="$base/deps"
inst="$deps/install"
export deps inst

mkdir "$deps" || true
rm -rf "$inst"

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"

osmo-build-dep.sh libosmocore
osmo-build-dep.sh libosmo-abis

cd "$deps"

# Get osmo-pcu for pcuif_proto.h
osmo-deps.sh osmo-pcu

# Get openbsc for gsm_data_shared.*
osmo-deps.sh openbsc

cd "$base"

set +x
echo
echo
echo
echo " =============================== osmo-bts-trx ==============================="
echo
set -x

autoreconf --install --force
configure_flags="\
  --with-openbsc=$deps/openbsc/openbsc/include \
  --with-osmo-pcu=$deps/osmo-pcu/include \
  --enable-trx \
  "
./configure $configure_flags
$MAKE $PARALLEL_MAKE
$MAKE check \
  || cat-testlogs.sh
DISTCHECK_CONFIGURE_FLAGS="$configure_flags" \
  $MAKE distcheck \
  || cat-testlogs.sh
