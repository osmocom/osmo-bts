AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOCODEC_CFLAGS) \
	$(LIBOSMOABIS_CFLAGS) $(LIBOSMOTRAU_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOCODEC_LIBS) \
	$(LIBOSMOABIS_LIBS) $(LIBOSMOTRAU_LIBS)
check_PROGRAMS = amr_test
EXTRA_DIST = amr_test.ok amr_test.err

amr_test_SOURCES = amr_test.c
amr_test_LDADD = $(top_builddir)/src/common/libbts.a \
		$(LDADD)
