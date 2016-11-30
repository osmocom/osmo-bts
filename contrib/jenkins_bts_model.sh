#!/bin/sh
bts_model="$1"

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
