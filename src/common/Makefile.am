AM_CPPFLAGS = \
	$(all_includes) \
	-I$(top_srcdir)/include \
	-I$(top_builddir)/include \
	$(NULL)

AM_CFLAGS = \
	-Wall \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOTRAU_CFLAGS) \
	$(LIBOSMOCODEC_CFLAGS) \
	$(LIBOSMONETIF_CFLAGS) \
	$(NULL)

LDADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOTRAU_LIBS) \
	$(LIBOSMOCODEC_LIBS) \
	$(LIBOSMONETIF_LIBS) \
	$(NULL)

if ENABLE_LC15BTS
AM_CFLAGS += -DENABLE_LC15BTS
endif

noinst_LIBRARIES = libbts.a libl1sched.a
libbts_a_SOURCES = \
	gsm_data.c \
	sysinfo.c \
	logging.c \
	abis.c \
	abis_osmo.c \
	oml.c \
	osmux.c \
	bts.c \
	bts_sm.c \
	bts_trx.c \
	rsl.c \
	rtp_abstract.c \
	rtp_input_preen.c \
	vty.c \
	paging.c \
	measurement.c \
	amr.c \
	asci.c \
	lchan.c \
	load_indication.c \
	pcu_sock.c \
	handover.c \
	msg_utils.c \
	tx_power.c \
	bts_ctrl_commands.c \
	bts_ctrl_lookup.c \
	bts_shutdown_fsm.c \
	csd_rlp.c \
	csd_v110.c \
	l1sap.c \
	cbch.c \
	power_control.c \
	main.c \
	phy_link.c \
	dtx_dl_amr_fsm.c \
	scheduler_mframe.c \
	ta_control.c \
	nm_common_fsm.c \
	nm_bts_sm_fsm.c \
	nm_bts_fsm.c \
	nm_bb_transc_fsm.c \
	nm_channel_fsm.c \
	nm_gprs_cell_fsm.c \
	nm_gprs_nse_fsm.c \
	nm_gprs_nsvc_fsm.c \
	nm_radio_carrier_fsm.c \
	notification.c \
	twrtp/bind_fdpair.c \
	twrtp/endp_bind.c \
	twrtp/endp_create.c \
	twrtp/endp_register.c \
	twrtp/rtcp_rx.c \
	twrtp/rtcp_tx.c \
	twrtp/rtp_rx.c \
	twrtp/rtp_tx.c \
	twrtp/set_dscp_prio.c \
	twrtp/set_remote.c \
	twrtp/set_sdes.c \
	twrtp/twjit.c \
	twrtp/twjit_in.c \
	twrtp/twjit_out.c \
	twrtp/twjit_vty.c \
	probes.d \
	$(NULL)

noinst_HEADERS = \
	twrtp/endp_internal.h \
	$(NULL)

libl1sched_a_SOURCES = scheduler.c

if ENABLE_SYSTEMTAP
probes.h: probes.d
	$(DTRACE) -C -h -s $< -o $@

probes.lo: probes.d
	$(LIBTOOL) --mode=compile $(AM_V_lt) --tag=CC env CFLAGS="$(CFLAGS)" $(DTRACE) -C -G -s $< -o $@

BUILT_SOURCES = probes.h probes.lo
libbts_la_LDADD = probes.lo
endif
