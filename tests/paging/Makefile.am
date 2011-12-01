INCLUDES = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(LIBOSMOTRAU_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOVTY_LIBS) $(LIBOSMOTRAU_LIBS) -lortp
noinst_PROGRAMS = paging_test
EXTRA_DIST = paging_test.ok

paging_test_SOURCES = paging_test.c
paging_test_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
