AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include -I$(OPENBSC_INCDIR) -I$(top_srcdir)/src/osmo-bts-sysmo
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(LIBOSMOTRAU_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOVTY_LIBS) $(LIBOSMOTRAU_LIBS) -lortp

noinst_PROGRAMS = sysmobts_test
EXTRA_DIST = sysmobts_test.ok

sysmobts_test_SOURCES = sysmobts_test.c $(top_srcdir)/src/osmo-bts-sysmo/utils.c \
		$(srcdir)/../stubs.c
sysmobts_test_LDADD = $(LDADD)
