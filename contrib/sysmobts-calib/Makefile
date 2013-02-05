CFLAGS=`pkg-config --cflags libosmocore` -Wall -Werror
LIBS=`pkg-config --libs libosmocore libosmogsm`

all: sysmobts-calib

sysmobts-calib: sysmobts-calib.o sysmobts-layer1.o
	$(CC) $(CPPFLAGS) -o $@ $^ -lrt $(LIBS)

clean:
	@rm -f sysmobts-calib *.o
