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
	$(LIBOSMOABIS_LIBS) \
	$(LIBOSMOTRAU_LIBS) \
	$(LIBOSMONETIF_LIBS) \
	$(NULL)
check_PROGRAMS = tx_power_test
EXTRA_DIST = tx_power_test.ok tx_power_test.err

tx_power_test_SOURCES = tx_power_test.c $(srcdir)/../stubs.c
tx_power_test_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
