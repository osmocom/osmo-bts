AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include -I$(OPENBSC_INCDIR)
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOCODEC_CFLAGS) $(LIBOSMOCODING_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOCODEC_LIBS) $(LIBOSMOCODING_LIBS)
noinst_PROGRAMS = bursts_test
EXTRA_DIST = bursts_test.ok

bursts_test_SOURCES = bursts_test.c
bursts_test_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
