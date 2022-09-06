AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS = \
	-Wall \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOVTY_CFLAGS) \
	$(LIBOSMOCODEC_CFLAGS) \
	$(LIBOSMOABIS_CFLAGS) \
	$(LIBOSMOTRAU_CFLAGS) \
	$(LIBOSMONETIF_CFAGS) \
	$(NULL)
LDADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(LIBOSMOCODEC_LIBS) \
	$(LIBOSMOABIS_LIBS) \
	$(LIBOSMOTRAU_LIBS) \
	$(LIBOSMONETIF_LIBS) \
	$(NULL)

check_PROGRAMS = agch_test
EXTRA_DIST = agch_test.ok

agch_test_SOURCES = agch_test.c $(srcdir)/../stubs.c
agch_test_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
