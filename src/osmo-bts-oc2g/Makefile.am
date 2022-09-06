AUTOMAKE_OPTIONS = subdir-objects

AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include $(OC2G_INCDIR)

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
	$(ORTP_CFLAGS) \
	$(NULL)

COMMON_LDADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOCODEC_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOABIS_LIBS) \
	$(LIBOSMOTRAU_LIBS) \
	$(LIBOSMONETIF_CFLAGS) \
	$(ORTP_LIBS) \
	$(NULL)

AM_CFLAGS += -DENABLE_OC2GBTS

EXTRA_DIST = \
	misc/oc2gbts_mgr.h \
	misc/oc2gbts_misc.h \
	misc/oc2gbts_par.h \
	misc/oc2gbts_led.h \
	misc/oc2gbts_temp.h \
	misc/oc2gbts_power.h \
	misc/oc2gbts_clock.h \
	misc/oc2gbts_bid.h \
	misc/oc2gbts_bts.h \
	misc/oc2gbts_nl.h \
	misc/oc2gbts_swd.h \
	hw_misc.h \
	l1_if.h \
	l1_transp.h \
	oc2gbts.h \
	utils.h \
	$(NULL)

bin_PROGRAMS = osmo-bts-oc2g oc2gbts-mgr oc2gbts-util

COMMON_SOURCES = \
	main.c \
	oc2gbts.c \
	l1_if.c \
	oml.c \
	oc2gbts_vty.c \
	tch.c \
	hw_misc.c \
	calib_file.c \
	utils.c \
	misc/oc2gbts_par.c \
	misc/oc2gbts_bid.c \
	$(NULL)

osmo_bts_oc2g_SOURCES = $(COMMON_SOURCES) l1_transp_hw.c
osmo_bts_oc2g_LDADD = \
	$(top_builddir)/src/common/libbts.a \
	$(COMMON_LDADD) \
	$(NULL)

oc2gbts_mgr_SOURCES = \
		misc/oc2gbts_mgr.c \
		misc/oc2gbts_misc.c \
		misc/oc2gbts_par.c \
		misc/oc2gbts_nl.c \
		misc/oc2gbts_temp.c \
		misc/oc2gbts_power.c \
		misc/oc2gbts_clock.c \
		misc/oc2gbts_bid.c \
		misc/oc2gbts_mgr_vty.c \
		misc/oc2gbts_mgr_nl.c \
		misc/oc2gbts_mgr_temp.c \
		misc/oc2gbts_mgr_calib.c \
		misc/oc2gbts_led.c \
		misc/oc2gbts_bts.c \
		misc/oc2gbts_swd.c \
		$(NULL)

oc2gbts_mgr_LDADD = \
	$(top_builddir)/src/common/libbts.a \
	$(COMMON_LDADD) \
	$(LIBGPS_LIBS) \
	$(LIBSYSTEMD_LIBS) \
	$(NULL)

oc2gbts_util_SOURCES = \
	misc/oc2gbts_util.c \
	misc/oc2gbts_par.c \
	$(NULL)
oc2gbts_util_LDADD = $(LIBOSMOCORE_LIBS)
