AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include $(OCTSDR2G_INCDIR)

AM_CFLAGS = \
	-Wall \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOCODEC_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOVTY_CFLAGS) \
	$(LIBOSMOCTRL_CFLAGS) \
	$(LIBOSMOABIS_CFLAGS) \
	$(LIBOSMOTRAU_CFLAGS) \
	$(LIBOSMONETIF_CFLAGS) \
	$(NULL)

COMMON_LDADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCODEC_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOABIS_LIBS) \
	$(LIBOSMOTRAU_LIBS) \
	$(LIBOSMONETIF_LIBS) \
	$(NULL)

EXTRA_DIST = l1_if.h l1_oml.h l1_utils.h octphy_hw_api.h octpkt.h

bin_PROGRAMS = osmo-bts-octphy

COMMON_SOURCES = \
	main.c \
	l1_if.c \
	l1_oml.c \
	l1_utils.c \
	l1_tch.c \
	octphy_hw_api.c \
	octphy_vty.c \
	octpkt.c \
	$(NULL)

osmo_bts_octphy_SOURCES = $(COMMON_SOURCES)
osmo_bts_octphy_LDADD = $(top_builddir)/src/common/libbts.a $(COMMON_LDADD)
