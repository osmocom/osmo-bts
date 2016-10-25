# XSL stylesheets downloaded from http://docbook.sourceforge.net/release/xsl/current/html/
# Makefile from BitBake/OpenEmbedded manuals

EXTRA_DEPS = gen-bts-vty-docbook

topdir = .
bts_reference = $(topdir)/osmobts-vty-reference.xml
manuals = $(bts_reference)
# types = pdf txt rtf ps xhtml html man tex texi dvi
# types = pdf txt
types = $(docbooktotypes)
docbooktotypes = pdf
# htmlcssfile =
# htmlcss =

TOPDIR := ..
ASCIIDOCS := osmobts-usermanual osmobts-abis

include $(TOPDIR)/build/Makefile.asciidoc.inc
include $(TOPDIR)/build/Makefile.inc

osmobts-usermanual.pdf: chapters/*.adoc

clean:
	-rm -rf $(cleanfiles)
	-rm osmobts-abis__*.png
	-rm osmobts-abis__*.svg
	-rm osmobts-usermanual__*.png
	-rm osmobts-usermanual__*.svg
	-rm osmobts-abis*.check
	-rm osmobts-usermanual*.check

gen-bts-vty-docbook: FORCE
	$(call command,xsltproc -o generated/combined1.xml \
		--stringparam with $(PWD)/../common/vty_additions.xml \
		$(MERGE_DOC) vty/bts_vty_reference.xml, \
		XSLTPROC,Merging Common VTY)
	$(call command,xsltproc -o generated/combined2.xml \
		--stringparam with $(PWD)/../common/ns_vty_additions.xml \
		$(MERGE_DOC) generated/combined1.xml, \
		XSLTPROC,Merging Common NS VTY)
	$(call command,xsltproc -o generated/combined3.xml \
		--stringparam with $(PWD)/vty/bts_vty_additions.xml \
		$(MERGE_DOC) generated/combined2.xml, \
		XSLTPROC,Merging BTS VTY)
	$(call command,xsltproc ../vty_reference.xsl generated/combined3.xml > generated/docbook_vty.xml, \
		XSLTPROC,Converting BTS VTY to DocBook)
