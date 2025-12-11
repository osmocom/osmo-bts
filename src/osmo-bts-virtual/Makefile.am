AM_CFLAGS = \
	-Wall -fno-strict-aliasing \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOVTY_CFLAGS) \
	$(LIBOSMOCTRL_CFLAGS) \
	$(LIBOSMOABIS_CFLAGS) \
	$(LIBOSMOTRAU_CFLAGS) \
	$(LIBOSMONETIF_CFLAGS) \
	$(LIBOSMOCODEC_CFLAGS) \
	$(NULL)

AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include -Iinclude

COMMON_LDADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOABIS_LIBS) \
	$(LIBOSMOTRAU_LIBS) \
	$(LIBOSMONETIF_LIBS) \
	$(LIBOSMOCODEC_LIBS) \
	-ldl \
	$(NULL)

noinst_HEADERS = \
	l1_if.h \
	osmo_mcast_sock.h \
	virtual_um.h \
	$(NULL)

bin_PROGRAMS = osmo-bts-virtual

osmo_bts_virtual_SOURCES = \
	main.c \
	bts_model.c \
	virtualbts_vty.c \
	scheduler_virtbts.c \
	l1_if.c \
	virtual_um.c \
	osmo_mcast_sock.c \
	$(NULL)

osmo_bts_virtual_LDADD = \
	$(top_builddir)/src/common/libl1sched.a \
	$(top_builddir)/src/common/libbts.a \
	$(COMMON_LDADD) \
	$(NULL)
