#!/bin/sh
# jenkins build helper script for osmo-bts-trx

# shellcheck source=contrib/jenkins_common.sh
. $(dirname "$0")/jenkins_common.sh

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"

osmo-build-dep.sh libosmocore

osmo-build-dep.sh libosmo-abis

cd "$deps"

# Get osmo-pcu for pcuif_proto.h
osmo-deps.sh osmo-pcu

configure_flags="\
  --with-osmo-pcu=$deps/osmo-pcu/include \
  --enable-trx \
  "

build_bts "osmo-bts-trx" "$configure_flags"

osmo-clean-workspace.sh
