dnl Process this file with autoconf to produce a configure script
AC_INIT([osmo-bts],
	m4_esyscmd([./git-version-gen .tarball-version]),
	[openbsc@lists.osmocom.org])

dnl *This* is the root dir, even if an install-sh exists in ../ or ../../
AC_CONFIG_AUX_DIR([.])

AM_INIT_AUTOMAKE([dist-bzip2])
AC_CONFIG_TESTDIR(tests)

CFLAGS="$CFLAGS -std=gnu11"

dnl kernel style compile messages
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

dnl include release helper
RELMAKE='-include osmo-release.mk'
AC_SUBST([RELMAKE])

dnl checks for programs
AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_INSTALL
LT_INIT

dnl patching ${archive_cmds} to affect generation of file "libtool" to fix linking with clang
AS_CASE(["$LD"],[*clang*],
  [AS_CASE(["${host_os}"],
     [*linux*],[archive_cmds='$CC -shared $pic_flag $libobjs $deplibs $compiler_flags $wl-soname $wl$soname -o $lib'])])

dnl check for pkg-config (explained in detail in libosmocore/configure.ac)
AC_PATH_PROG(PKG_CONFIG_INSTALLED, pkg-config, no)
if test "x$PKG_CONFIG_INSTALLED" = "xno"; then
        AC_MSG_WARN([You need to install pkg-config])
fi
PKG_PROG_PKG_CONFIG([0.20])

dnl checks for header files
AC_HEADER_STDC

dnl Checks for typedefs, structures and compiler characteristics

AC_ARG_ENABLE(sanitize,
		[AS_HELP_STRING([--enable-sanitize], [Compile with address sanitizer enabled], )],
		[sanitize=$enableval], [sanitize="no"])
if test x"$sanitize" = x"yes"
then
	CFLAGS="$CFLAGS -fsanitize=address -fsanitize=undefined"
	CPPFLAGS="$CPPFLAGS -fsanitize=address -fsanitize=undefined"
fi

AC_ARG_ENABLE(werror,
	[AS_HELP_STRING(
		[--enable-werror],
		[Turn all compiler warnings into errors, with exceptions:
		 a) deprecation (allow upstream to mark deprecation without breaking builds);
		 b) "#warning" pragmas (allow to remind ourselves of errors without breaking builds)
		]
	)],
	[werror=$enableval], [werror="no"])
if test x"$werror" = x"yes"
then
	WERROR_FLAGS="-Werror"
	WERROR_FLAGS+=" -Wno-error=deprecated -Wno-error=deprecated-declarations"
	WERROR_FLAGS+=" -Wno-error=cpp" # "#warning"
	CFLAGS="$CFLAGS $WERROR_FLAGS"
	CPPFLAGS="$CPPFLAGS $WERROR_FLAGS"
fi

dnl checks for libraries
PKG_CHECK_MODULES(LIBOSMOCORE, libosmocore >= 1.7.0)
PKG_CHECK_MODULES(LIBOSMOVTY, libosmovty >= 1.7.0)
PKG_CHECK_MODULES(LIBOSMOGSM, libosmogsm >= 1.7.0)
PKG_CHECK_MODULES(LIBOSMOCTRL, libosmoctrl >= 1.7.0)
PKG_CHECK_MODULES(LIBOSMOCODEC, libosmocodec >= 1.7.0)
PKG_CHECK_MODULES(LIBOSMOCODING, libosmocoding >= 1.7.0)
PKG_CHECK_MODULES(LIBOSMOABIS, libosmoabis >= 1.3.0)
PKG_CHECK_MODULES(LIBOSMOTRAU, libosmotrau >= 1.3.0)

AC_MSG_CHECKING([whether to enable support for sysmobts calibration tool])
AC_ARG_ENABLE(sysmobts-calib,
		AC_HELP_STRING([--enable-sysmobts-calib],
				[enable code for sysmobts calibration tool [default=no]]),
		[enable_sysmobts_calib="yes"],[enable_sysmobts_calib="no"])
AC_MSG_RESULT([$enable_sysmobts_calib])
AM_CONDITIONAL(ENABLE_SYSMOBTS_CALIB, test "x$enable_sysmobts_calib" = "xyes")

AC_MSG_CHECKING([whether to enable support for sysmoBTS L1/PHY support])
AC_ARG_ENABLE(sysmocom-bts,
		AC_HELP_STRING([--enable-sysmocom-bts],
				[enable code for sysmoBTS L1/PHY [default=no]]),
		[enable_sysmocom_bts="yes"],[enable_sysmocom_bts="no"])
AC_ARG_WITH([sysmobts], [AS_HELP_STRING([--with-sysmobts=INCLUDE_DIR], [Location of the sysmobts API header files, implies --enable-sysmocom-bts])],
			 [sysmobts_incdir="$withval"],[sysmobts_incdir="$incdir"])
if test "x$sysmobts_incdir" != "x"; then
	# --with-sysmobts was passed, imply enable_sysmocom_bts
	enable_sysmocom_bts="yes"
fi
AC_SUBST([SYSMOBTS_INCDIR], -I$sysmobts_incdir)
AC_MSG_RESULT([$enable_sysmocom_bts])
AM_CONDITIONAL(ENABLE_SYSMOBTS, test "x$enable_sysmocom_bts" = "xyes")
if test "$enable_sysmocom_bts" = "yes"; then
	oldCPPFLAGS=$CPPFLAGS
	CPPFLAGS="$CPPFLAGS $SYSMOBTS_INCDIR -I$srcdir/include"
	AC_CHECK_HEADER([sysmocom/femtobts/superfemto.h],[],
			[AC_MSG_ERROR([sysmocom/femtobts/superfemto.h can not be found in $sysmobts_incdir])],
			[#include <sysmocom/femtobts/superfemto.h>])

	# Check for the sbts2050_header.h that was added after the 3.6 release
	AC_CHECK_HEADER([sysmocom/femtobts/sbts2050_header.h], [sysmo_uc_header="yes"], [])
	if test "$sysmo_uc_header" = "yes" ; then
	   	AC_DEFINE(BUILD_SBTS2050, 1, [Define if we want to build SBTS2050])
	fi

	PKG_CHECK_MODULES(LIBGPS, libgps)
	CPPFLAGS=$oldCPPFLAGS
fi
AM_CONDITIONAL(BUILD_SBTS2050, test "x$sysmo_uc_header" = "xyes")

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
AC_SUBST([OCTSDR2G_INCDIR], -I$octsdr2g_incdir)
AC_MSG_RESULT([$enable_octphy])
AM_CONDITIONAL(ENABLE_OCTPHY, test "x$enable_octphy" = "xyes")
if test "$enable_octphy" = "yes" ; then
	oldCPPFLAGS=$CPPFLAGS
	CPPFLAGS="$CPPFLAGS $OCTSDR2G_INCDIR -I$srcdir/include"

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

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_INFO_RSP.ulClkSourceSelection],
			AC_DEFINE([OCTPHY_USE_CLK_SOURCE_SELECTION],
			[1],
			[Define to 1 if your octphy header files supports ulClkSourceSelection in tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_INFO_RSP]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP.lClockError],
			AC_DEFINE([OCTPHY_USE_CLOCK_SYNC_MGR_STATS_CLOCK_ERROR],
			[1],
			[Define to 1 if your octphy header files supports lClockError in tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP.lDroppedCycles],
			AC_DEFINE([OCTPHY_USE_CLOCK_SYNC_MGR_STATS_DROPPED_CYCLES],
			[1],
			[Define to 1 if your octphy header files supports lDroppedCycles in tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP.ulPllFreqHz],
			AC_DEFINE([OCTPHY_USE_CLOCK_SYNC_MGR_STATS_PLL_FREQ_HZ],
			[1],
			[Define to 1 if your octphy header files supports ulPllFreqHz in tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP.ulPllFractionalFreqHz],
			AC_DEFINE([OCTPHY_USE_CLOCK_SYNC_MGR_STATS_PLL_FRACTIONAL_FREQ_HZ],
			[1],
			[Define to 1 if your octphy header files supports ulPllFractionalFreqHz in tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP.ulSlipCnt],
			AC_DEFINE([OCTPHY_USE_CLOCK_SYNC_MGR_STATS_SLIP_CNT],
			[1],
			[Define to 1 if your octphy header files supports ulSlipCnt in tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP.ulSyncLosseCnt],
			AC_DEFINE([OCTPHY_USE_CLOCK_SYNC_MGR_STATS_SYNC_LOSSE_CNT],
			[1],
			[Define to 1 if your octphy header files supports ulSyncLosseCnt in tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP.ulSyncLossCnt],
			AC_DEFINE([OCTPHY_USE_CLOCK_SYNC_MGR_STATS_SYNC_LOSS_CNT],
			[1],
			[Define to 1 if your octphy header files supports ulSyncLossCnt in tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP.ulSourceState],
			AC_DEFINE([OCTPHY_USE_CLOCK_SYNC_MGR_STATS_SOURCE_STATE],
			[1],
			[Define to 1 if your octphy header files supports ulSourceState in tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP.ulDacState],
			AC_DEFINE([OCTPHY_USE_CLOCK_SYNC_MGR_STATS_DAC_STATE],
			[1],
			[Define to 1 if your octphy header files supports ulDacState in tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP.ulDriftElapseTimeUs],
			AC_DEFINE([OCTPHY_USE_CLOCK_SYNC_MGR_STATS_DRIFT_ELAPSE_TIME_US],
			[1],
			[Define to 1 if your octphy header files supports ulDriftElapseTimeUs in tOCTVC1_HW_MSG_CLOCK_SYNC_MGR_STATS_RSP]),
			[],
			[#include <octphy/octvc1/hw/octvc1_hw_api.h>])

	AC_CHECK_MEMBER([tOCTVC1_GSM_MSG_OVERSAMPLE_SELECT_16X_MODIFY_CMD.ulOversample16xEnableFlag],
			AC_DEFINE([OCTPHY_USE_16X_OVERSAMPLING],
			[1],
			[Define to 1 if your octphy header files support 16x oversampling]),
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
AC_SUBST([LITECELL15_INCDIR], -I$litecell15_incdir)
AC_MSG_RESULT([$enable_litecell15])
AM_CONDITIONAL(ENABLE_LC15BTS, test "x$enable_litecell15" = "xyes")
if test "$enable_litecell15" = "yes"; then
	oldCPPFLAGS=$CPPFLAGS
	CPPFLAGS="$CPPFLAGS $LITECELL15_INCDIR -I$srcdir/include"
	AC_CHECK_HEADER([nrw/litecell15/litecell15.h],[],
			[AC_MSG_ERROR([nrw/litecell15/litecell15.h can not be found in $litecell15_incdir])],
			[#include <nrw/litecell15/litecell15.h>])
	PKG_CHECK_MODULES(LIBSYSTEMD, libsystemd)
	CPPFLAGS=$oldCPPFLAGS
fi

AC_MSG_CHECKING([whether to enable NuRAN Wireless OC-2G hardware support])
AC_ARG_ENABLE(oc2g,
		AC_HELP_STRING([--enable-oc2g],
				[enable code for NuRAN Wireless OC-2G bts [default=no]]),
		[enable_oc2g="yes"],[enable_oc2g="no"])
AC_ARG_WITH([oc2g], [AS_HELP_STRING([--with-oc2g=INCLUDE_DIR], [Location of the OC-2G API header files])],
			 [oc2g_incdir="$withval"],[oc2g_incdir="$incdir"])
AC_SUBST([OC2G_INCDIR], -I$oc2g_incdir)
AC_MSG_RESULT([$enable_oc2g])
AM_CONDITIONAL(ENABLE_OC2GBTS, test "x$enable_oc2g" = "xyes")
if test "$enable_oc2g" = "yes"; then
	oldCPPFLAGS=$CPPFLAGS
	CPPFLAGS="$CPPFLAGS $OC2G_INCDIR -I$srcdir/include"
	AC_CHECK_HEADER([nrw/oc2g/oc2g.h],[],
			[AC_MSG_ERROR([nrw/oc2g/oc2g.h can not be found in $oc2g_incdir])],
			[#include <nrw/oc2g/oc2g.h>])
	PKG_CHECK_MODULES(LIBSYSTEMD, libsystemd)
        PKG_CHECK_MODULES(LIBGPS, libgps)
	CPPFLAGS=$oldCPPFLAGS
fi

# Generate manuals
AC_ARG_ENABLE(manuals,
	[AS_HELP_STRING(
		[--enable-manuals],
		[Generate manual PDFs [default=no]],
	)],
	[osmo_ac_build_manuals=$enableval], [osmo_ac_build_manuals="no"])
AM_CONDITIONAL([BUILD_MANUALS], [test x"$osmo_ac_build_manuals" = x"yes"])
AC_ARG_VAR(OSMO_GSM_MANUALS_DIR, [path to common osmo-gsm-manuals files, overriding pkg-config and "../osmo-gsm-manuals"
	fallback])
if test x"$osmo_ac_build_manuals" = x"yes"
then
	# Find OSMO_GSM_MANUALS_DIR (env, pkg-conf, fallback)
	if test -n "$OSMO_GSM_MANUALS_DIR"; then
		echo "checking for OSMO_GSM_MANUALS_DIR... $OSMO_GSM_MANUALS_DIR (from env)"
	else
		OSMO_GSM_MANUALS_DIR="$($PKG_CONFIG osmo-gsm-manuals --variable=osmogsmmanualsdir 2>/dev/null)"
		if test -n "$OSMO_GSM_MANUALS_DIR"; then
			echo "checking for OSMO_GSM_MANUALS_DIR... $OSMO_GSM_MANUALS_DIR (from pkg-conf)"
		else
			OSMO_GSM_MANUALS_DIR="../osmo-gsm-manuals"
			echo "checking for OSMO_GSM_MANUALS_DIR... $OSMO_GSM_MANUALS_DIR (fallback)"
		fi
	fi
	if ! test -d "$OSMO_GSM_MANUALS_DIR"; then
		AC_MSG_ERROR("OSMO_GSM_MANUALS_DIR does not exist! Install osmo-gsm-manuals or set OSMO_GSM_MANUALS_DIR.")
	fi

	# Find and run check-depends
	CHECK_DEPENDS="$OSMO_GSM_MANUALS_DIR/check-depends.sh"
	if ! test -x "$CHECK_DEPENDS"; then
		CHECK_DEPENDS="osmo-gsm-manuals-check-depends"
	fi
	if ! $CHECK_DEPENDS; then
		AC_MSG_ERROR("missing dependencies for --enable-manuals")
	fi

	# Put in Makefile with absolute path
	OSMO_GSM_MANUALS_DIR="$(realpath "$OSMO_GSM_MANUALS_DIR")"
	AC_SUBST([OSMO_GSM_MANUALS_DIR])
fi

AC_ARG_ENABLE([external_tests],
		AC_HELP_STRING([--enable-external-tests],
				[Include the VTY/CTRL tests in make check [default=no]]),
		[enable_ext_tests="$enableval"],[enable_ext_tests="no"])
if test "x$enable_ext_tests" = "xyes" ; then
	AC_CHECK_PROG(PYTHON3_AVAIL,python3,yes)
	 if test "x$PYTHON3_AVAIL" != "xyes" ; then
		AC_MSG_ERROR([Please install python3 to run the VTY/CTRL tests.])
	fi
	AC_CHECK_PROG(OSMOTESTEXT_CHECK,osmotestvty.py,yes)
	 if test "x$OSMOTESTEXT_CHECK" != "xyes" ; then
		AC_MSG_ERROR([Please install https://gitea.osmocom.org/cellular-infrastructure/osmo-python-tests to run the VTY/CTRL tests.])
	fi
fi
AC_MSG_CHECKING([whether to enable VTY/CTRL tests])
AC_MSG_RESULT([$enable_ext_tests])
AM_CONDITIONAL(ENABLE_EXT_TESTS, test "x$enable_ext_tests" = "xyes")

#
# SystemTap support
#
AC_MSG_CHECKING([whether to include systemtap tracing support])
AC_ARG_ENABLE([systemtap],
	      [AS_HELP_STRING([--enable-systemtap],
			      [Enable inclusion of systemtap trace support])],
	      [ENABLE_SYSTEMTAP="${enableval}"], [ENABLE_SYSTEMTAP='no'])
AM_CONDITIONAL([ENABLE_SYSTEMTAP], [test x$ENABLE_SYSTEMTAP = xyes])
AC_MSG_RESULT(${ENABLE_SYSTEMTAP})

if test "x${ENABLE_SYSTEMTAP}" = xyes; then
  # Additional configuration for --enable-systemtap is HERE
  AC_CHECK_PROGS(DTRACE, dtrace)
  if test -z "$DTRACE"; then
    AC_MSG_ERROR([dtrace not found])
  fi
  AC_CHECK_HEADER([sys/sdt.h], [SDT_H_FOUND='yes'],
                [SDT_H_FOUND='no';
                   AC_MSG_ERROR([systemtap support needs sys/sdt.h header])])
  AC_DEFINE([HAVE_SYSTEMTAP], [1], [Define to 1 if using SystemTap probes.])
  AC_ARG_WITH([tapset-install-dir],
	      [AS_HELP_STRING([--with-tapset-install-dir],
	         [The absolute path where the tapset dir will be installed])],
	      [if test "x${withval}" = x; then
		 ABS_TAPSET_DIR="\$(datadir)/systemtap/tapset"
	       else
		 ABS_TAPSET_DIR="${withval}"
	       fi], [ABS_TAPSET_DIR="\$(datadir)/systemtap/tapset"])
  AC_SUBST(ABS_TAPSET_DIR)
fi

# https://www.freedesktop.org/software/systemd/man/daemon.html
AC_ARG_WITH([systemdsystemunitdir],
     [AS_HELP_STRING([--with-systemdsystemunitdir=DIR], [Directory for systemd service files])],,
     [with_systemdsystemunitdir=auto])
AS_IF([test "x$with_systemdsystemunitdir" = "xyes" -o "x$with_systemdsystemunitdir" = "xauto"], [
     def_systemdsystemunitdir=$($PKG_CONFIG --variable=systemdsystemunitdir systemd)

     AS_IF([test "x$def_systemdsystemunitdir" = "x"],
   [AS_IF([test "x$with_systemdsystemunitdir" = "xyes"],
    [AC_MSG_ERROR([systemd support requested but pkg-config unable to query systemd package])])
    with_systemdsystemunitdir=no],
   [with_systemdsystemunitdir="$def_systemdsystemunitdir"])])
AS_IF([test "x$with_systemdsystemunitdir" != "xno"],
      [AC_SUBST([systemdsystemunitdir], [$with_systemdsystemunitdir])])
AM_CONDITIONAL([HAVE_SYSTEMD], [test "x$with_systemdsystemunitdir" != "xno"])

AC_MSG_RESULT([CFLAGS="$CFLAGS"])
AC_MSG_RESULT([CPPFLAGS="$CPPFLAGS"])

AM_CONFIG_HEADER(btsconfig.h)

AC_OUTPUT(
    src/Makefile
    src/common/Makefile
    src/osmo-bts-virtual/Makefile
    src/osmo-bts-omldummy/Makefile
    src/osmo-bts-sysmo/Makefile
    src/osmo-bts-lc15/Makefile
    src/osmo-bts-oc2g/Makefile
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
    tests/ta_control/Makefile
    tests/tx_power/Makefile
    tests/power/Makefile
    tests/meas/Makefile
    tests/amr/Makefile
    doc/Makefile
    doc/examples/Makefile
    doc/manuals/Makefile
    contrib/Makefile
    contrib/systemd/Makefile
    contrib/osmo-bts.spec
    Makefile)
