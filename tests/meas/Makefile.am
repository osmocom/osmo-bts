AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS = \
	-Wall \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOCODEC_CFLAGS) \
	$(LIBOSMOABIS_CFLAGS) \
	$(LIBOSMOTRAU_CFLAGS) \
	$(LIBOSMONETIF_CFLAGS) \
	$(NULL)
AM_LDFLAGS = -no-install
LDADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOCODEC_LIBS) \
	$(LIBOSMOTRAU_LIBS) \
	$(LIBOSMOABIS_LIBS) \
	$(LIBOSMONETIF_LIBS) \
	$(NULL)

check_PROGRAMS = meas_test
noinst_HEADERS = sysmobts_fr_samples.h meas_testcases.h
EXTRA_DIST = meas_test.ok meas_test.err

meas_test_SOURCES = meas_test.c $(srcdir)/../stubs.c
meas_test_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
