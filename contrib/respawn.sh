#!/bin/sh
while [ -e /etc/passwd ]; do
	$*
done
