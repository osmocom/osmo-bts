AUTOMAKE_OPTIONS = foreign dist-bzip2 1.6

SUBDIRS = include src tests doc contrib


# package the contrib and doc
EXTRA_DIST = \
	contrib/dump_docs.py \
	git-version-gen .version \
	README.md

AM_DISTCHECK_CONFIGURE_FLAGS = \
	--with-systemdsystemunitdir=$$dc_install_base/$(systemdsystemunitdir)

@RELMAKE@

BUILT_SOURCES = $(top_srcdir)/.version

$(top_srcdir)/.version:
	echo $(VERSION) > $@-t && mv $@-t $@
dist-hook:
	echo $(VERSION) > $(distdir)/.tarball-version
