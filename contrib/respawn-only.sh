#!/bin/sh
while [ -f $1 ]; do
	nice -n -20 $*
done
