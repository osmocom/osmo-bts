AUTOMAKE_OPTIONS = subdir-objects

AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include $(LITECELL15_INCDIR)

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
	$(LIBGPS_CFLAGS) \
	$(LIBSYSTEMD_CFLAGS) \
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

AM_CFLAGS += -DENABLE_LC15BTS

EXTRA_DIST = \
	misc/lc15bts_mgr.h \
	misc/lc15bts_misc.h \
	misc/lc15bts_par.h \
	misc/lc15bts_led.h \
	misc/lc15bts_temp.h \
	misc/lc15bts_power.h \
	misc/lc15bts_clock.h \
	misc/lc15bts_bid.h \
	misc/lc15bts_nl.h \
	misc/lc15bts_bts.h \
	misc/lc15bts_swd.h \
	hw_misc.h \
	l1_if.h \
	l1_transp.h \
	lc15bts.h \
	utils.h \
	$(NULL)

bin_PROGRAMS = osmo-bts-lc15 lc15bts-mgr lc15bts-util

COMMON_SOURCES = \
	main.c \
	lc15bts.c \
	l1_if.c \
	oml.c \
	lc15bts_vty.c \
	tch.c \
	hw_misc.c \
	calib_file.c \
	utils.c \
	misc/lc15bts_par.c \
	misc/lc15bts_bid.c \
	$(NULL)

osmo_bts_lc15_SOURCES = $(COMMON_SOURCES) l1_transp_hw.c
osmo_bts_lc15_LDADD = \
	$(top_builddir)/src/common/libbts.a \
	$(COMMON_LDADD) \
	$(NULL)

lc15bts_mgr_SOURCES = \
		misc/lc15bts_mgr.c \
		misc/lc15bts_misc.c \
		misc/lc15bts_par.c \
		misc/lc15bts_nl.c \
		misc/lc15bts_temp.c \
		misc/lc15bts_power.c \
		misc/lc15bts_clock.c \
		misc/lc15bts_bid.c \
		misc/lc15bts_mgr_vty.c \
		misc/lc15bts_mgr_nl.c \
		misc/lc15bts_mgr_temp.c \
		misc/lc15bts_mgr_calib.c \
		misc/lc15bts_led.c \
		misc/lc15bts_bts.c \
		misc/lc15bts_swd.c \
		$(NULL)

lc15bts_mgr_LDADD = \
	$(top_builddir)/src/common/libbts.a \
	$(COMMON_LDADD) \
	$(LIBGPS_LIBS) \
	$(LIBSYSTEMD_LIBS) \
	$(NULL)

lc15bts_util_SOURCES = \
	misc/lc15bts_util.c \
	misc/lc15bts_par.c \
	$(NULL)
lc15bts_util_LDADD = $(LIBOSMOCORE_LIBS)
