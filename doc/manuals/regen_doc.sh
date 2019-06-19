#!/bin/sh -x

if [ -z "$DOCKER_PLAYGROUND" ]; then
	echo "You need to set DOCKER_PLAYGROUND"
	exit 1
fi

SCRIPT=$(realpath "$0")
MANUAL_DIR=$(dirname "$SCRIPT")

COMMIT=${COMMIT:-$(git log -1 --format=format:%H)}

cd "$DOCKER_PLAYGROUND/scripts" || exit 1

OSMO_BTS_BRANCH=$COMMIT ./regen_doc.sh osmo-bts 4241 \
	"$MANUAL_DIR/chapters/counters_generated.adoc" \
	"$MANUAL_DIR/vty/bts_vty_reference.xml"
