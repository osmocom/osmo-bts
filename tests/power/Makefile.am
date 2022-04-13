AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOCODEC_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(LIBOSMOTRAU_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOCODEC_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOVTY_LIBS) $(LIBOSMOTRAU_LIBS)

check_PROGRAMS = ms_power_loop_test bs_power_loop_test
EXTRA_DIST = ms_power_loop_test.ok ms_power_loop_test.err \
	     bs_power_loop_test.ok bs_power_loop_test.err

ms_power_loop_test_SOURCES = ms_power_loop_test.c $(srcdir)/../stubs.c
ms_power_loop_test_LDADD = $(top_builddir)/src/common/libbts.a $(LIBOSMOABIS_LIBS) $(LDADD)

bs_power_loop_test_SOURCES = bs_power_loop_test.c $(srcdir)/../stubs.c
bs_power_loop_test_LDADD = $(top_builddir)/src/common/libbts.a $(LIBOSMOABIS_LIBS) $(LDADD)
