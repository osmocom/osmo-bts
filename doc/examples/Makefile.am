
doc_virtualdir = $(docdir)/examples/osmo-bts-virtual
doc_virtual_DATA = \
        virtual/osmobts-virtual.cfg \
        virtual/openbsc-virtual.cfg
EXTRA_DIST = $(doc_virtual_DATA)

if ENABLE_SYSMOBTS
doc_sysmodir = $(docdir)/examples/osmo-bts-sysmo
doc_sysmo_DATA = \
        sysmo/osmo-bts.cfg \
        sysmo/sysmobts-mgr.cfg
EXTRA_DIST += $(doc_sysmo_DATA)
endif

if ENABLE_TRX
doc_trxdir = $(docdir)/examples/osmo-bts-trx
doc_trx_DATA = \
        trx/osmo-bts.cfg \
        trx/osmo-bts-trx-calypso.cfg
EXTRA_DIST += $(doc_trx_DATA)
endif

if ENABLE_OCTPHY
doc_octphydir = $(docdir)/examples/osmo-bts-octphy
doc_octphy_DATA = \
        octphy/osmo-bts-trx2dsp1.cfg \
        octphy/osmo-bts.cfg
EXTRA_DIST += $(doc_octphy_DATA)
endif

if ENABLE_LC15BTS
doc_lc15dir = $(docdir)/examples/osmo-bts-lc15
doc_lc15_DATA = \
        litecell15/osmo-bts.cfg \
        litecell15/lc15bts-mgr.cfg
EXTRA_DIST += $(doc_lc15_DATA)
endif
