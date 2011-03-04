INCLUDES = $(all_includes) -I$(top_srcdir)/include
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS)

noinst_LIBRARIES = libbts.a
libbts_a_SOURCES = support.c bts.c abis.c rsl.c oml.c rtp.c
#	../common/l1l2_interface.c ../common/lapdm.c ../common/logging.c
