#!/bin/sh
while [ -e /etc/passwd ]; do
	cat /lib/firmware/sysmobts-v?.out > /dev/dspdl_dm644x_0
	$*
done
