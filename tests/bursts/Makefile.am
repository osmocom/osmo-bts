INCLUDES = $(all_includes) -I$(top_srcdir)/include -I$(OPENBSC_INCDIR)
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOCODEC_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOCODEC_LIBS)
noinst_PROGRAMS = bursts_test
EXTRA_DIST = bursts_test.ok

bursts_test_SOURCES = bursts_test.c \
			$(top_builddir)/src/osmo-bts-trx/gsm0503_coding.c \
			$(top_builddir)/src/osmo-bts-trx/gsm0503_conv.c \
			$(top_builddir)/src/osmo-bts-trx/gsm0503_interleaving.c \
			$(top_builddir)/src/osmo-bts-trx/gsm0503_mapping.c \
			$(top_builddir)/src/osmo-bts-trx/gsm0503_tables.c \
			$(top_builddir)/src/osmo-bts-trx/gsm0503_parity.c
bursts_test_LDADD = $(LDADD)
