AM_CPPFLAGS = \
	$(all_includes) \
	-I$(top_srcdir)/include \
	-I$(top_builddir)/include \
	$(NULL)

AM_CFLAGS = \
	-Wall -fno-strict-aliasing \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOCODEC_CFLAGS) \
	$(LIBOSMOCODING_CFLAGS) \
	$(LIBOSMOVTY_CFLAGS) \
	$(LIBOSMOCTRL_CFLAGS) \
	$(LIBOSMOABIS_CFLAGS) \
	$(LIBOSMOTRAU_CFLAGS) \
	$(LIBOSMONETIF_CFLAGS) \
	$(NULL)

LDADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOCODEC_LIBS) \
	$(LIBOSMOCODING_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	$(LIBOSMOABIS_LIBS) \
	$(LIBOSMOTRAU_LIBS) \
	$(LIBOSMONETIF_LIBS) \
	-ldl \
	$(NULL)

noinst_HEADERS = \
	sched_utils.h \
	trx_if.h \
	l1_if.h \
	amr_loop.h \
	trx_provision_fsm.h \
	$(NULL)

bin_PROGRAMS = osmo-bts-trx

osmo_bts_trx_SOURCES = \
	main.c \
	trx_if.c \
	l1_if.c \
	scheduler_trx.c \
	sched_lchan_fcch_sch.c \
	sched_lchan_rach.c \
	sched_lchan_xcch.c \
	sched_lchan_pdtch.c \
	sched_lchan_tchf.c \
	sched_lchan_tchh.c \
	trx_provision_fsm.c \
	trx_vty.c \
	amr_loop.c \
	probes.d \
	$(NULL)

osmo_bts_trx_LDADD = \
	$(top_builddir)/src/common/libl1sched.a \
	$(top_builddir)/src/common/libbts.a \
	$(LDADD) \
	$(NULL)

if ENABLE_SYSTEMTAP
probes.h: probes.d
	$(DTRACE) -C -h -s $< -o $@

probes.lo: probes.d
	$(LIBTOOL) --mode=compile $(AM_V_lt) --tag=CC env CFLAGS="$(CFLAGS)" $(DTRACE) -C -G -s $< -o $@

BUILT_SOURCES = probes.h probes.lo
osmo_bts_trx_LDADD += probes.lo
endif
