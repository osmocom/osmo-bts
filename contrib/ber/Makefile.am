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

rtp_ber_SOURCES = rtp_ber.c codec_bit_class.h

rtp_gen_map_SOURCES = rtp_gen_map.c

update_codec_bit_class_h: rtp_gen_map
	$(AM_V_GEN)./$< > $(top_srcdir)/contrib/ber/codec_bit_class.h
