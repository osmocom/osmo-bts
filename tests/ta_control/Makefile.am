AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOCODEC_CFLAGS) $(LIBOSMOTRAU_CFLAGS) $(LIBOSMOABIS_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOCODEC_LIBS) $(LIBOSMOTRAU_LIBS) $(LIBOSMOABIS_LIBS)
check_PROGRAMS = ta_control_test
EXTRA_DIST = ta_control_test.ok
ta_control_test_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
