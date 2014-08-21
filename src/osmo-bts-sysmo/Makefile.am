AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include -I$(OPENBSC_INCDIR)
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(LIBOSMOTRAU_CFLAGS) $(LIBOSMOABIS_CFLAGS)
COMMON_LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOVTY_LIBS) $(LIBOSMOTRAU_LIBS) $(LIBOSMOABIS_LIBS) -lortp

EXTRA_DIST = misc/sysmobts_mgr.h misc/sysmobts_misc.h misc/sysmobts_par.h \
	misc/sysmobts_eeprom.h misc/sysmobts_nl.h femtobts.h hw_misc.h \
	l1_fwd.h l1_if.h l1_transp.h eeprom.h utils.h oml_router.h

bin_PROGRAMS = sysmobts sysmobts-remote l1fwd-proxy sysmobts-mgr sysmobts-util

COMMON_SOURCES = main.c femtobts.c l1_if.c oml.c sysmobts_vty.c tch.c hw_misc.c calib_file.c \
		 eeprom.c calib_fixup.c utils.c misc/sysmobts_par.c oml_router.c

sysmobts_SOURCES = $(COMMON_SOURCES) l1_transp_hw.c
sysmobts_LDADD = $(top_builddir)/src/common/libbts.a $(COMMON_LDADD)

sysmobts_remote_SOURCES = $(COMMON_SOURCES) l1_transp_fwd.c
sysmobts_remote_LDADD = $(top_builddir)/src/common/libbts.a $(COMMON_LDADD)

l1fwd_proxy_SOURCES = l1_fwd_main.c l1_transp_hw.c
l1fwd_proxy_LDADD = $(top_builddir)/src/common/libbts.a $(COMMON_LDADD)

sysmobts_mgr_SOURCES = \
		misc/sysmobts_mgr.c misc/sysmobts_misc.c \
		misc/sysmobts_par.c misc/sysmobts_nl.c \
		misc/sysmobts_mgr_2050.c \
		misc/sysmobts_mgr_vty.c
sysmobts_mgr_LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOVTY_LIBS)

sysmobts_util_SOURCES = misc/sysmobts_util.c misc/sysmobts_par.c
sysmobts_util_LDADD = $(LIBOSMOCORE_LIBS)
