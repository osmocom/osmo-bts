AM_CPPFLAGS = $(all_includes) -I$(top_srcdir)/include -I$(OPENBSC_INCDIR)
AM_CFLAGS = -D_GNU_SOURCE -Wall $(LIBOSMOCORE_CFLAGS) $(LIBOSMOGSM_CFLAGS) $(LIBOSMOVTY_CFLAGS) $(LIBOSMOTRAU_CFLAGS) $(ORTP_CFLAGS)
LDADD = $(LIBOSMOCORE_LIBS) $(LIBOSMOGSM_LIBS) $(LIBOSMOVTY_LIBS) $(LIBOSMOTRAU_LIBS) $(LIBOSMOABIS_LIBS) $(ORTP_LIBS)
noinst_PROGRAMS = client_bidir client_mcast client_unix server_bidir server_mcast server_unix virt_um_bts virt_um_ms
# EXTRA_DIST = client_bidir.ok client_mcast.ok client_unix.ok server_bidir.ok server_mcast.ok server_unix.ok virt_um_bts.ok virt_um_bts.ok

client_bidir_SOURCES = virtsock_client_mcast_bidir.c mcast_sock.c
#client_bidir_LDADD = $(top_builddir)/src/common/libbts.a $(LDADD)
server_bidir_SOURCES = virtsock_server_mcast_bidir.c mcast_sock.c
client_mcast_SOURCES = virtsock_client_mcast.c 
server_mcast_SOURCES = virtsock_server_mcast.c 
client_unix_SOURCES = virtsock_client_unix_domain.c 
server_unix_SOURCES = virtsock_server_unix_domain.c 
virt_um_ms_SOURCES = virt_um_ms.c $(top_srcdir)/src/osmo-bts-virtual/virtual_um.c $(top_srcdir)/src/osmo-bts-virtual/osmo_mcast_sock.c
virt_um_bts_SOURCES = virt_um_bts.c $(top_srcdir)/src/osmo-bts-virtual/virtual_um.c $(top_srcdir)/src/osmo-bts-virtual/osmo_mcast_sock.c
