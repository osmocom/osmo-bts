osmo-bts - Osmocom BTS Implementation
====================================

This repository contains a C-language implementation of a GSM Base
Transceiver Station (BTS). It is part of the
[Osmocom](https://osmocom.org/) Open Source Mobile Communications
project.

This code implements Layer 2 and higher of a more or less conventional GSM BTS
(Base Transceiver Station) - however, using an Abis/IP interface, rather than
the old-fashioned E1/T1.

Specifically, this includes
 * BTS-side implementation of TS 08.58 (RSL) and TS 12.21 (OML)
 * BTS-side implementation of LAPDm (using libosmocore/libosmogsm)
 * A somewhat separated interface between those higher layer parts and the
   Layer1 interface.

Several kinds of BTS hardware are supported:
 * sysmocom sysmoBTS
 * Octasic octphy
 * Nutaq litecell 1.5
 * OpenCellular 2G (OC-2G)
 * software-defined radio based osmo-bts-trx (e.g. USRP B210, UmTRX, LimeSDR)

Homepage
--------

The official homepage of the project is
<https://osmocom.org/projects/osmobts/wiki>

GIT Repository
--------------

You can clone from the official osmo-bts.git repository using

	git clone https://gitea.osmocom.org/cellular-infrastructure/osmo-bts

There is a web interface at <https://gitea.osmocom.org/cellular-infrastructure/osmo-bts>

Documentation
-------------

User Manuals and VTY reference manuals are [optionally] built in PDF form
as part of the build process.

Pre-rendered PDF version of the current "master" can be found at
[User Manual](https://ftp.osmocom.org/docs/latest/osmobts-usermanual.pdf)
as well as the VTY reference manuals
* [VTY Reference Manual for osmo-bts-sysmo](https://ftp.osmocom.org/docs/latest/osmobts-sysmo-vty-reference.pdf)
* [VTY Reference Manual for osmo-bts-trx](https://ftp.osmocom.org/docs/latest/osmobts-trx-vty-reference.pdf)
* [VTY Reference Manual for osmo-bts-lc15](https://ftp.osmocom.org/docs/latest/osmobts-lc15-vty-reference.pdf)
* [VTY Reference Manual for osmo-bts-oc2g](https://ftp.osmocom.org/docs/latest/osmobts-oc2g-vty-reference.pdf)
* [VTY Reference Manual for osmo-bts-octphy](https://ftp.osmocom.org/docs/latest/osmobts-octphy-vty-reference.pdf)

There also is an
[Abis reference Manual](https://ftp.osmocom.org/docs/latest/osmobts-abis.pdf)
describing the OsmoBTS specific A-bis dialect.

Forum
-----

We welcome any osmo-bts related discussions in the
[Cellular Network Infrastructure -> 2G RAN (GERAN)](https://discourse.osmocom.org/c/cni/geran)
section of the osmocom discourse (web based Forum).

Mailing List
------------

Discussions related to osmo-bts are happening on the
openbsc@lists.osmocom.org mailing list, please see
https://lists.osmocom.org/mailman/listinfo/openbsc for subscription
options and the list archive.

Please observe the [Osmocom Mailing List
Rules](https://osmocom.org/projects/cellular-infrastructure/wiki/Mailing_List_Rules)
when posting.

Issue Tracker
-------------

We use the [issue tracker of the osmo-bts project on osmocom.org](https://osmocom.org/projects/osmobts/issues) for
tracking the state of bug reports and feature requests.  Feel free to submit any issues you may find, or help
us out by resolving existing issues.

Contributing
------------

Our coding standards are described at
https://osmocom.org/projects/cellular-infrastructure/wiki/Coding_standards

We use a Gerrit based patch submission/review process for managing
contributions.  Please see
https://osmocom.org/projects/cellular-infrastructure/wiki/Gerrit for
more details

The current patch queue for osmo-bts can be seen at
https://gerrit.osmocom.org/#/q/project:osmo-bts+status:open

Known Limitations
=================

As of January 2021, the following known limitations exist in this
implementation:

Common Core
-----------

 * No Extended BCCH support
 * System Information limited to 1,2,2bis,2ter,2quater,3,4,5,6,9,13
 * No RATSCCH in AMR
 * Will reject TS 12.21 STARTING TIME in SET BTS ATTR / SET CHAN ATTR
 * No support of TS 08.58 MultiRate Control
 * No support of TS 08.58 Supported Codec Types
 * No support of Bter frame / ENHANCED MEASUREMENT REPORT

osmo-bts-sysmo
--------------

 * No CSD / ECSD support (not planned)
 * GSM-R frequency band supported, but no NCH/ASCI/SoLSA
 * All timeslots on one TRX have to use same training sequence (TSC)
 * No multi-TRX support yet, though hardware+L1 support stacking
 * Makes no use of 12.21 Intave Parameters and Interference
   Level Boundaries
 * MphConfig.CNF can be returned to the wrong callback. E.g. with Tx Power
   and ciphering. The dispatch should take a look at the hLayer3.

osmo-bts-octphy
---------------

 * No support of EFR, HR voice codec (lack of PHY support?)
 * No re-transmission of PHY primitives in case of time-out
 * Link Quality / Measurement processing incomplete
 * impossible to modify encryption parameters using RSL MODE MODIFY
 * no clear indication of nominal transmit power, various power related
   computations are likely off
 * no OML attribute validation during bts_model_check_oml()
