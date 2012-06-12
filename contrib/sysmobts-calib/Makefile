CFLAGS=`pkg-config --cflags libosmocore` -Wall -Werror

all: sysmobts-calib

sysmobts-calib: sysmobts-calib.o sysmobts-layer1.o
	$(CC) $(CPPFLAGS) -o $@ $^ -lrt

clean:
	@rm -f sysmobts-calib *.o
