EXTRA_DIST = \
  lc15bts-mgr.service \
  oc2gbts-mgr.service \
  osmo-bts-lc15.service \
  osmo-bts-oc2g.service \
  osmo-bts-sysmo.service \
  osmo-bts-trx.service \
  osmo-bts-virtual.service \
  sysmobts-mgr.service

if HAVE_SYSTEMD
SYSTEMD_SERVICES = osmo-bts-virtual.service


if ENABLE_SYSMOBTS
SYSTEMD_SERVICES += osmo-bts-sysmo.service sysmobts-mgr.service
endif

if ENABLE_TRX
SYSTEMD_SERVICES += osmo-bts-trx.service
endif

if ENABLE_LC15BTS
SYSTEMD_SERVICES += osmo-bts-lc15.service lc15bts-mgr.service
endif

if ENABLE_OC2GBTS
SYSTEMD_SERVICES += osmo-bts-oc2g.service oc2gbts-mgr.service
endif

systemdsystemunit_DATA = $(SYSTEMD_SERVICES)
endif # HAVE_SYSTEMD
