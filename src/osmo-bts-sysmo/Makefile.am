INCLUDES = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(LIBOSMOTRAU_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOVTY_LIBS) $(LIBOSMOTRAU_LIBS)

bin_PROGRAMS = sysmobts sysmobts-remote l1fwd-proxy

COMMON_SOURCES = main.c femtobts.c l1_if.c oml.c bts_model.c sysmobts_vty.c tch.c

sysmobts_SOURCES = $(COMMON_SOURCES) l1_transp_hw.c
sysmobts_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)

sysmobts_remote_SOURCES = $(COMMON_SOURCES) l1_transp_fwd.c
sysmobts_remote_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)

l1fwd_proxy_SOURCES = l1_fwd_main.c l1_transp_hw.c
l1fwd_proxy_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
