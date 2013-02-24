INCLUDES = $(all_includes) -I$(top_srcdir)/include -I$(OPENBSC_INCDIR)
AM_CFLAGS = -Wall -fno-strict-aliasing $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(LIBOSMOTRAU_CFLAGS) $(LIBOSMOABIS_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOVTY_LIBS) $(LIBOSMOTRAU_LIBS) $(LIBOSMOABIS_LIBS) -lortp

EXTRA_DIST = trx_if.h l1_if.h scheduler.h gsm0503_parity.h gsm0503_conv.h gsm0503_interleaving.h gsm0503_mapping.h gsm0503_coding.h gsm0503_tables.h

bin_PROGRAMS = osmobts-trx

osmobts_trx_SOURCES = main.c trx_if.c l1_if.c scheduler.c trx_vty.c gsm0503_parity.c gsm0503_conv.c gsm0503_interleaving.c gsm0503_mapping.c gsm0503_coding.c gsm0503_tables.c
osmobts_trx_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)

