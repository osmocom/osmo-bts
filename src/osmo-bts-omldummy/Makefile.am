AM_CFLAGS = \
	-Wall -fno-strict-aliasing \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOVTY_CFLAGS) \
	$(LIBOSMOCTRL_CFLAGS) \
	$(LIBOSMOABIS_CFLAGS) \
	$(LIBOSMOTRAU_CFLAGS) \
	$(LIBGPS_CFLAGS) \
	$(NULL)

AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include -Iinclude

COMMON_LDADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOABIS_LIBS) \
	$(LIBOSMOTRAU_LIBS) \
	-ldl \
	$(NULL)

bin_PROGRAMS = osmo-bts-omldummy

osmo_bts_omldummy_SOURCES = main.c bts_model.c
osmo_bts_omldummy_LDADD = $(top_builddir)/src/common/libbts.a $(COMMON_LDADD)
