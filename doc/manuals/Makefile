TOPDIR = ..

ASCIIDOC = osmobts-usermanual.adoc osmobts-abis.adoc rtp-amr.adoc
include $(TOPDIR)/build/Makefile.asciidoc.inc
osmobts-usermanual.pdf: chapters/*.adoc
osmobts-abis.pdf: abis/*.adoc abis/*.msc
rtp-amr.pdf: dtx.dot

VTY_REFERENCE = osmobts-vty-reference.xml
include $(TOPDIR)/build/Makefile.vty-reference.inc

include $(TOPDIR)/build/Makefile.common.inc
