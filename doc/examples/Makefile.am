# all config examples must be listed here unconditionally, so that
# all of them end up in the release tarball (see OS#6349)
EXTRA_DIST = \
	trx/osmo-bts-trx.cfg \
	trx/osmo-bts-trx-calypso.cfg \
	octphy/osmo-bts-trx2dsp1.cfg \
	octphy/osmo-bts-octphy.cfg \
	oc2g/osmo-bts-oc2g.cfg \
	oc2g/oc2gbts-mgr.cfg \
	sysmo/sysmobts-mgr.cfg \
	sysmo/osmo-bts-sysmo.cfg \
	litecell15/osmo-bts-lc15.cfg \
	litecell15/lc15bts-mgr.cfg \
	virtual/osmo-bts-virtual.cfg \
	$(NULL)

doc_virtualdir = $(docdir)/examples/osmo-bts-virtual
doc_virtual_DATA = \
        virtual/osmo-bts-virtual.cfg
OSMOCONF_FILES = virtual/osmo-bts-virtual.cfg

if ENABLE_SYSMOBTS
doc_sysmodir = $(docdir)/examples/osmo-bts-sysmo
doc_sysmo_DATA = \
        sysmo/osmo-bts-sysmo.cfg \
        sysmo/sysmobts-mgr.cfg
OSMOCONF_FILES += sysmo/osmo-bts-sysmo.cfg sysmo/sysmobts-mgr.cfg
endif

if ENABLE_TRX
doc_trxdir = $(docdir)/examples/osmo-bts-trx
doc_trx_DATA = \
        trx/osmo-bts-trx.cfg \
        trx/osmo-bts-trx-calypso.cfg
OSMOCONF_FILES += trx/osmo-bts-trx.cfg
endif

if ENABLE_OCTPHY
doc_octphydir = $(docdir)/examples/osmo-bts-octphy
doc_octphy_DATA = \
        octphy/osmo-bts-trx2dsp1.cfg \
        octphy/osmo-bts-octphy.cfg
OSMOCONF_FILES += octphy/osmo-bts-octphy.cfg
endif

if ENABLE_LC15BTS
doc_lc15dir = $(docdir)/examples/osmo-bts-lc15
doc_lc15_DATA = \
        litecell15/osmo-bts-lc15.cfg \
        litecell15/lc15bts-mgr.cfg
OSMOCONF_FILES += litecell15/osmo-bts-lc15.cfg litecell15/lc15bts-mgr.cfg
endif

if ENABLE_OC2GBTS
doc_oc2gdir = $(docdir)/examples/osmo-bts-oc2g
doc_oc2g_DATA = \
        oc2g/osmo-bts-oc2g.cfg \
        oc2g/oc2gbts-mgr.cfg
OSMOCONF_FILES += oc2g/osmo-bts-oc2g.cfg oc2g/oc2gbts-mgr.cfg
endif

osmoconfdir = $(sysconfdir)/osmocom
osmoconf_DATA = $(OSMOCONF_FILES)
