#!/bin/sh
# jenkins build helper script for osmo-bts.  This is how we build on jenkins.osmocom.org
#
# environment variables:
# * FIRMWARE_VERSION: which firmware version to build ("master", "femtobts_v2.7", ...)
# * WITH_MANUALS: build manual PDFs if set to "1"
# * PUBLISH: upload manuals after building if set to "1" (ignored without WITH_MANUALS = "1")
#
# usage: jenkins_bts_model.sh BTS_MODEL
# * BTS_MODEL: which BTS model specific script to run ("sysmo", "oct", ...)
#

bts_model="$1"

if [ "x$bts_model" = "x" ]; then
	echo "Error: You have to specify the BTS model as first argument, e.g. $0 sysmo"
	exit 2
fi

if [ ! -d "./contrib" ]; then
  echo "Run ./contrib/jenkins_bts_model.sh from the root of the osmo-bts tree"
  exit 1
fi

set -x -e

case "$bts_model" in

  sysmo)
    ./contrib/jenkins_sysmobts.sh
  ;;

  oct)
    ./contrib/jenkins_oct.sh
  ;;

  lc15)
    ./contrib/jenkins_lc15.sh
  ;;

  oc2g)
    ./contrib/jenkins_oc2g.sh
  ;;

  trx)
    ./contrib/jenkins_bts_trx.sh
  ;;

  oct+trx)
    ./contrib/jenkins_oct_and_bts_trx.sh
  ;;

  *)
    set +x
    echo "Unknown BTS model '$bts_model'"
  ;;
esac
