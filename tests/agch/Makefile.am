AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include -I$(OPENBSC_INCDIR)
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(LIBOSMOTRAU_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOVTY_LIBS) $(LIBOSMOTRAU_LIBS) $(LIBOSMOABIS_LIBS) -lortp
noinst_PROGRAMS = agch_test
EXTRA_DIST = agch_test.ok

agch_test_SOURCES = agch_test.c $(srcdir)/../stubs.c
agch_test_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
