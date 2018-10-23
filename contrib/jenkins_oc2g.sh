#!/bin/sh
# jenkins build helper script for osmo-bts-oc2g

# shellcheck source=contrib/jenkins_common.sh
. $(dirname "$0")/jenkins_common.sh

osmo-build-dep.sh libosmocore "" --disable-doxygen

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"

osmo-build-dep.sh libosmo-abis

cd "$deps"
osmo-layer1-headers.sh oc2g "$FIRMWARE_VERSION"

configure_flags="\
  --enable-sanitize \
  --with-oc2g=$deps/layer1-headers/inc/ \
  --enable-oc2g \
  "

build_bts "osmo-bts-oc2g" "$configure_flags"

osmo-clean-workspace.sh
