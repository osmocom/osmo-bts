#!/bin/sh

# shellcheck source=contrib/jenkins_common.sh
. $(dirname "$0")/jenkins_common.sh

osmo-build-dep.sh libosmocore

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"

osmo-build-dep.sh libosmo-abis

cd "$deps"
if ! test -d litecell15-fw;
then
  git clone https://gitlab.com/nrw_litecell15/litecell15-fw.git
fi

cd litecell15-fw
git fetch origin
git reset --hard origin/$FIRMWARE_VERSION

cd "$base"

set +x
echo
echo
echo
echo " =============================== osmo-bts-lc15 ==============================="
echo
set -x

autoreconf --install --force
./configure --with-openbsc="$deps/openbsc/openbsc/include" --with-litecell15="$deps/litecell15-fw/" --enable-litecell15
$MAKE "$PARALLEL_MAKE"
$MAKE check || cat-testlogs.sh
DISTCHECK_CONFIGURE_FLAGS="--with-litecell15=$deps/litecell15-fw/ --with-openbsc=$deps/openbsc/openbsc/include --enable-litecell15" $MAKE distcheck || cat-testlogs.sh
