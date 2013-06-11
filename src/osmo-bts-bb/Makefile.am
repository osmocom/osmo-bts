AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS)

bin_PROGRAMS = osmo-bts-bb

osmo_bts_bb_SOURCES = main.c
osmo_bts_bb_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
