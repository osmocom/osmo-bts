dnl Process this file with autoconf to produce a configure script
AC_INIT([osmo-bts],
	m4_esyscmd([./git-version-gen .tarball-version]),
	[openbsc-devel@lists.openbsc.org])

dnl *This* is the root dir, even if an install-sh exists in ../ or ../../
AC_CONFIG_AUX_DIR([.])

AM_INIT_AUTOMAKE([dist-bzip2])
AC_CONFIG_TESTDIR(tests)

dnl kernel style compile messages
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

dnl include release helper
RELMAKE='-include osmo-release.mk'
AC_SUBST([RELMAKE])

dnl checks for programs
AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_RANLIB

dnl check for pkg-config (explained in detail in libosmocore/configure.ac)
AC_PATH_PROG(PKG_CONFIG_INSTALLED, pkg-config, no)
if test "x$PKG_CONFIG_INSTALLED" = "xno"; then
        AC_MSG_WARN([You need to install pkg-config])
fi
PKG_PROG_PKG_CONFIG([0.20])

dnl checks for header files
AC_HEADER_STDC

dnl Checks for typedefs, structures and compiler characteristics

dnl checks for libraries
PKG_CHECK_MODULES(LIBOSMOCORE, libosmocore  >= 0.3.9)
PKG_CHECK_MODULES(LIBOSMOVTY, libosmovty)
PKG_CHECK_MODULES(LIBOSMOTRAU, libosmotrau >= 0.3.2)
PKG_CHECK_MODULES(LIBOSMOGSM, libosmogsm >= 0.3.3)
PKG_CHECK_MODULES(LIBOSMOCTRL, libosmoctrl)
PKG_CHECK_MODULES(LIBOSMOABIS, libosmoabis)
PKG_CHECK_MODULES(LIBOSMOCODEC, libosmocodec)
PKG_CHECK_MODULES(LIBOSMOCODING, libosmocoding)
PKG_CHECK_MODULES(ORTP, ortp)

AC_MSG_CHECKING([whether to enable support for sysmoBTS L1/PHY support])
AC_ARG_ENABLE(sysmocom-bts,
		AC_HELP_STRING([--enable-sysmocom-bts],
				[enable code for sysmoBTS L1/PHY [default=no]]),
		[enable_sysmocom_bts="yes"],[enable_sysmocom_bts="no"])
AC_MSG_RESULT([$enable_sysmocom_bts])
AM_CONDITIONAL(ENABLE_SYSMOBTS, test "x$enable_sysmocom_bts" = "xyes")
if test "$enable_sysmocom_bts" = "yes"; then
	PKG_CHECK_MODULES(LIBGPS, libgps)
fi

AC_MSG_CHECKING([whether to enable support for osmo-trx based L1/PHY support])
AC_ARG_ENABLE(trx,
		AC_HELP_STRING([--enable-trx],
				[enable code for osmo-trx L1/PHY [default=no]]),
		[enable_trx="yes"],[enable_trx="no"])
AC_MSG_RESULT([$enable_trx])
AM_CONDITIONAL(ENABLE_TRX, test "x$enable_trx" = "xyes")

AC_MSG_CHECKING([whether to enable support for Octasic OCTPHY-2G])
AC_ARG_ENABLE(octphy,
		AC_HELP_STRING([--enable-octphy],
				[enable code for Octasic OCTPHY-2G [default=no]]),
		[enable_octphy="yes"],[enable_octphy="no"])
AC_ARG_WITH([octsdr-2g], [AS_HELP_STRING([--with-octsdr-2g], [Location of the OCTSDR-2G API header files])],
			 [octsdr2g_incdir="$withval"],[octsdr2g_incdir="`cd $srcdir; pwd`/src/osmo-bts-octphy/"])
AC_SUBST([OCTSDR2G_INCDIR], $octsdr2g_incdir)
AC_MSG_RESULT([$enable_octphy])
AM_CONDITIONAL(ENABLE_OCTPHY, test "x$enable_octphy" = "xyes")
if test "$enable_octphy" = "yes" ; then
	oldCPPFLAGS=$CPPFLAGS
	CPPFLAGS="$CPPFLAGS -I$OCTSDR2G_INCDIR -I$srcdir/include $LIBOSMOCORE_CFLAGS"

	AC_CHECK_HEADER([octphy/octvc1/gsm/octvc1_gsm_default.h],[],
			[AC_MSG_ERROR([octphy/octvc1/gsm/octvc1_gsm_default.h can not be found in $octsdr2g_incdir])],
			[#include <octphy/octvc1/gsm/octvc1_gsm_default.h>])

	AC_CHECK_MEMBER([tOCTVC1_GSM_TRX_CONFIG.usCentreArfcn],
			AC_DEFINE([OCTPHY_MULTI_TRX],
			[1],
			[Define to 1 if your octphy header files support multi-trx]),
			[],
			[#include <octphy/octvc1/gsm/octvc1_gsm_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_RF_PORT_RX_STATS.Frequency],
			AC_DEFINE([OCTPHY_USE_FREQUENCY],
			[1],
			[Define to 1 if your octphy header files support tOCTVC1_RADIO_FREQUENCY_VALUE type]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP.ulSyncLossCnt],
			AC_DEFINE([OCTPHY_USE_SYNC_LOSS_CNT],
			[1],
			[Define to 1 if your octphy header files renamed ulSyncLosseCnt to ulSyncLossCnt]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_RF_PORT_INFO_ANTENNA_TX_CONFIG_RSP.TxConfig],
			AC_DEFINE([OCTPHY_USE_TX_CONFIG],
			[1],
			[Define to 1 if your octphy header files support tOCTVC1_HW_RF_PORT_ANTENNA_TX_CONFIG type]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_RF_PORT_INFO_ANTENNA_RX_CONFIG_RSP.RxConfig],
			AC_DEFINE([OCTPHY_USE_RX_CONFIG],
			[1],
			[Define to 1 if your octphy header files support tOCTVC1_HW_RF_PORT_ANTENNA_RX_CONFIG type]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_GSM_RF_CONFIG.ulTxAntennaId],
			AC_DEFINE([OCTPHY_USE_ANTENNA_ID],
			[1],
			[Define to 1 if your octphy header files support antenna ids in tOCTVC1_GSM_RF_CONFIG]),
			[],
			[#include <octphy/octvc1/gsm/octvc1_gsm_api.h>])

	CPPFLAGS=$oldCPPFLAGS
fi

AC_MSG_CHECKING([whether to enable NuRAN Wireless Litecell 1.5 hardware support])
AC_ARG_ENABLE(litecell15,
		AC_HELP_STRING([--enable-litecell15],
				[enable code for NuRAN Wireless Litecell15 bts [default=no]]),
		[enable_litecell15="yes"],[enable_litecell15="no"])
AC_ARG_WITH([litecell15], [AS_HELP_STRING([--with-litecell15=INCLUDE_DIR], [Location of the litecell 1.5 API header files])],
			 [litecell15_incdir="$withval"],[litecell15_incdir="$incdir"])
AC_SUBST([LITECELL15_INCDIR], $litecell15_incdir)
AC_MSG_RESULT([$enable_litecell15])
AM_CONDITIONAL(ENABLE_LC15BTS, test "x$enable_litecell15" = "xyes")
if test "$enable_litecell15" = "yes"; then
	oldCPPFLAGS=$CPPFLAGS
	CPPFLAGS="$CPPFLAGS -I$LITECELL15_INCDIR -I$srcdir/include $LIBOSMOCORE_CFLAGS"
	AC_CHECK_HEADER([nrw/litecell15/litecell15.h],[],
			[AC_MSG_ERROR([nrw/litecell15/litecell15.h can not be found in $litecell15_incdir])],
			[#include <nrw/litecell15/litecell15.h>])
	PKG_CHECK_MODULES(LIBSYSTEMD, libsystemd)
	CPPFLAGS=$oldCPPFLAGS
fi

# We share gsm_data.h with OpenBSC and need to be pointed to the source
# directory of OpenBSC for now.
AC_ARG_WITH([openbsc],
	[AS_HELP_STRING([--with-openbsc=INCLUDE_DIR],
		[OpenBSC include directory for openbsc/gsm_data_shared.h])],
	[openbsc_incdir="$withval"],
	[openbsc_incdir="`cd $srcdir; pwd`/../openbsc/openbsc/include"])
AC_SUBST([OPENBSC_INCDIR], $openbsc_incdir)

oldCPPFLAGS=$CPPFLAGS
CPPFLAGS="$CPPFLAGS -I$OPENBSC_INCDIR -I$srcdir/include $LIBOSMOCORE_CFLAGS"
AC_CHECK_HEADER([openbsc/gsm_data_shared.h],[],
	[AC_MSG_ERROR([openbsc/gsm_data_shared.h can not be found in $openbsc_incdir])],
	[#include <osmo-bts/tx_power.h>])
CPPFLAGS=$oldCPPFLAGS

# Check for the sbts2050_header.h that was added after the 3.6 release
oldCPPFLAGS=$CPPFLAGS
CPPFLAGS="$CPPFLAGS -I$OPENBSC_INCDIR $LIBOSMOCORE_CFLAGS"
AC_CHECK_HEADER([sysmocom/femtobts/sbts2050_header.h],
		[sysmo_uc_header="yes"],[])
CPPFLAGS=$oldCPPFLAGS

if test "$sysmo_uc_header" = "yes" ; then
	AC_DEFINE(BUILD_SBTS2050, 1, [Define if we want to build SBTS2050])
fi
AM_CONDITIONAL(BUILD_SBTS2050, test "x$sysmo_uc_header" = "xyes")
AM_CONFIG_HEADER(btsconfig.h)

AC_OUTPUT(
    src/Makefile
    src/common/Makefile
    src/osmo-bts-virtual/Makefile
    src/osmo-bts-sysmo/Makefile
    src/osmo-bts-litecell15/Makefile
    src/osmo-bts-trx/Makefile
    src/osmo-bts-octphy/Makefile
    include/Makefile
    include/osmo-bts/Makefile
    tests/Makefile
    tests/paging/Makefile
    tests/agch/Makefile
    tests/cipher/Makefile
    tests/sysmobts/Makefile
    tests/misc/Makefile
    tests/handover/Makefile
    tests/tx_power/Makefile
    tests/meas/Makefile
    Makefile)
