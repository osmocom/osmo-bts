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

check_PROGRAMS = csd_test
EXTRA_DIST = csd_test.err

csd_test_SOURCES = csd_test.c
csd_test_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
