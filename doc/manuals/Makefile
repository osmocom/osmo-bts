TOPDIR := ..
ASCIIDOCS := osmobts-usermanual osmobts-abis

include $(TOPDIR)/build/Makefile.asciidoc.inc

# for the upload target
include $(TOPDIR)/build/Makefile.inc

# dependencies
osmobts-abis.pdf: abis/*.adoc abis/*.msc
osmobts-usermanual.pdf: chapters/*.adoc

clean:
	rm -rf $(cleanfiles)
