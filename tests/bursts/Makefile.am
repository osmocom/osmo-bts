INCLUDES = $(all_includes) -I$(top_srcdir)/include -I$(OPENBSC_INCDIR)
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS)
noinst_PROGRAMS = bursts_test
EXTRA_DIST = bursts_test.ok

bursts_test_SOURCES = bursts_test.c \
			$(top_builddir)/src/osmo-bts-trx/xcch.c \
			$(top_builddir)/src/osmo-bts-trx/rach.c \
			$(top_builddir)/src/osmo-bts-trx/sch.c \
			$(top_builddir)/src/osmo-bts-trx/tch_fr.c \
			$(top_builddir)/src/osmo-bts-trx/pxxch.c
bursts_test_LDADD = $(LDADD)
