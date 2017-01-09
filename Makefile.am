AUTOMAKE_OPTIONS = foreign dist-bzip2 1.6

SUBDIRS = include src tests


# package the contrib and doc
EXTRA_DIST = \
	contrib/dump_docs.py contrib/screenrc-l1fwd contrib/osmo-bts-sysmo.service \
	contrib/l1fwd.init contrib/screenrc-sysmobts contrib/respawn.sh \
	contrib/sysmobts.init contrib/sysmobts-calib/Makefile \
	contrib/sysmobts-calib/sysmobts-calib.c \
	contrib/sysmobts-calib/sysmobts-layer1.c \
	contrib/sysmobts-calib/sysmobts-layer1.h \
	doc/examples/sysmo/osmo-bts.cfg \
	doc/examples/sysmobts-mgr.cfg \
	doc/examples/virtual/openbsc-virtual.cfg \
	doc/examples/virtual/osmobts-virtual.cfg \
	README.md

@RELMAKE@
