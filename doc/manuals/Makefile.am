EXTRA_DIST = dtx.dot \
    osmobts-abis.adoc \
    osmobts-abis-docinfo.xml \
    osmobts-usermanual.adoc \
    osmobts-usermanual-docinfo.xml \
    osmobts-vty-reference.xml \
    rtp-amr.adoc \
    rtp-amr-docinfo.xml \
    regen_doc.sh \
    abis \
    chapters \
    vty

if BUILD_MANUALS
  ASCIIDOC = osmobts-usermanual.adoc osmobts-abis.adoc rtp-amr.adoc
  include $(OSMO_GSM_MANUALS_DIR)/build/Makefile.asciidoc.inc
  osmobts-usermanual.pdf: $(srcdir)/chapters/*.adoc
  osmobts-abis.pdf: $(srcdir)/abis/*.adoc $(srcdir)/abis/*.msc
  rtp-amr.pdf: $(srcdir)/dtx.dot

  VTY_REFERENCE = osmobts-vty-reference.xml
  include $(OSMO_GSM_MANUALS_DIR)/build/Makefile.vty-reference.inc

  OSMO_REPOSITORY = osmo-bts
  include $(OSMO_GSM_MANUALS_DIR)/build/Makefile.common.inc
endif
