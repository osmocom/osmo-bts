AM_CPPFLAGS = \
	$(all_includes) \
	-I$(top_srcdir)/include \
	$(NULL)

AM_CFLAGS = \
	-Wall -fno-strict-aliasing \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOGSM_CFLAGS) \
	$(LIBOSMOCODEC_CFLAGS) \
	$(LIBOSMOCODING_CFLAGS) \
	$(LIBOSMOVTY_CFLAGS) \
	$(LIBOSMOTRAU_CFLAGS) \
	$(LIBOSMOABIS_CFLAGS) \
	$(LIBOSMOCTRL_CFLAGS) \
	$(NULL)

LDADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOGSM_LIBS) \
	$(LIBOSMOCODEC_LIBS) \
	$(LIBOSMOCODING_LIBS) \
	$(LIBOSMOVTY_LIBS) \
	$(LIBOSMOTRAU_LIBS) \
	$(LIBOSMOABIS_LIBS) \
	$(LIBOSMOCTRL_LIBS) \
	-ldl \
	$(NULL)

noinst_HEADERS = \
	sched_utils.h \
	trx_if.h \
	l1_if.h \
	loops.h \
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
	loops.c \
	$(NULL)

osmo_bts_trx_LDADD = \
	$(top_builddir)/src/common/libl1sched.a \
	$(top_builddir)/src/common/libbts.a \
	$(LDADD) \
	$(NULL)
