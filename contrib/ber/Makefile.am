AM_CPPFLAGS = \
	$(all_includes) \
	-I$(top_srcdir)/include \
	-I$(top_builddir)/include \
	-I$(builddir) \
	$(NULL)

AM_CFLAGS = \
	-Wall \
	$(LIBOSMOCORE_CFLAGS) \
	$(LIBOSMOTRAU_CFLAGS) \
	$(LIBOSMOCODEC_CFLAGS) \
	$(NULL)

LDADD = \
	$(LIBOSMOCORE_LIBS) \
	$(LIBOSMOTRAU_LIBS) \
	$(LIBOSMOCODEC_LIBS) \
	$(NULL)

noinst_PROGRAMS = rtp_ber rtp_gen_map

BUILT_SOURCES = codec_bit_class.h
DISTCLEANFILES = codec_bit_class.h

codec_bit_class.h: rtp_gen_map
	$(AM_V_GEN)./$< > $@

rtp_ber_SOURCES = rtp_ber.c codec_bit_class.h

rtp_gen_map_SOURCES = rtp_gen_map.c
