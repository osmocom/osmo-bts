INCLUDES = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOVTY_LIBS)

bin_PROGRAMS = osmobts

osmobts_SOURCES = main.c oml.c bts_model.c l1_if.c l1ctl.c
osmobts_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
