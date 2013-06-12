AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include -I$(OPENBSC_INCDIR)
AM_CFLAGS = -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOTRAU_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOTRAU_LIBS)

noinst_LIBRARIES = libbts.a
libbts_a_SOURCES = gsm_data_shared.c sysinfo.c logging.c abis.c oml.c bts.c \
		   rsl.c vty.c paging.c measurement.c amr.c lchan.c \
		   load_indication.c pcu_sock.c handover.c