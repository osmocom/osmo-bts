EXTRA_DIST = dtx.dot \
    osmobts-abis.adoc \
    osmobts-abis-docinfo.xml \
    osmobts-usermanual.adoc \
    osmobts-usermanual-docinfo.xml \
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

  # NOTE: osmo-bts-omldummy has no VTY interface
  VARIANTS = virtual

if ENABLE_SYSMOBTS
  VARIANTS += sysmo
endif

if ENABLE_TRX
  VARIANTS += trx
endif

if ENABLE_OCTPHY
  VARIANTS += octphy
endif

if ENABLE_LC15BTS
  VARIANTS += lc15
endif

if ENABLE_OC2GBTS
  VARIANTS += oc2g
endif

  # This is a significantly modified, multi-target adopted copy of
  # $(OSMO_GSM_MANUALS_DIR)/build/Makefile.vty-reference.inc
  include $(srcdir)/vty/Makefile.vty-reference.inc

  OSMO_REPOSITORY = osmo-bts
  include $(OSMO_GSM_MANUALS_DIR)/build/Makefile.common.inc
endif
