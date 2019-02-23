CC=gcc
LD=gcc
CFLAGS=`pkg-config --cflags libosmocore libosmotrau libosmocodec` -O2 -Wall -ggdb
LDLIBS=`pkg-config --libs   libosmocore libosmotrau libosmocodec` -ggdb

all: rtp_ber

rtp_ber.o: codec_bit_class.h

codec_bit_class.h: rtp_gen_map
	./rtp_gen_map > codec_bit_class.h

clean:
	rm -f rtp_ber rtp_gen_map codec_bit_class.h *.o
