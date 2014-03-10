dnl Process this file with autoconf to produce a configure script
AC_INIT([osmo-bts],
	m4_esyscmd([./git-version-gen .tarball-version]),
	[openbsc-devel@lists.openbsc.org])

AM_INIT_AUTOMAKE([dist-bzip2])
AC_CONFIG_TESTDIR(tests)

dnl kernel style compile messages
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

dnl checks for programs
AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_RANLIB

dnl checks for header files
AC_HEADER_STDC

dnl Checks for typedefs, structures and compiler characteristics

dnl checks for libraries
PKG_CHECK_MODULES(LIBOSMOCORE, libosmocore  >= 0.3.9)
PKG_CHECK_MODULES(LIBOSMOVTY, libosmovty)
PKG_CHECK_MODULES(LIBOSMOTRAU, libosmotrau >= 0.0.7)
PKG_CHECK_MODULES(LIBOSMOGSM, libosmogsm >= 0.3.3)
PKG_CHECK_MODULES(LIBOSMOABIS, libosmoabis)

AC_MSG_CHECKING([whether to enable sysmocom-bts hardware support])
AC_ARG_ENABLE(sysmocom-bts,
		AC_HELP_STRING([--enable-sysmocom-bts],
				[enable code for sysmocom femto-bts [default=no]]),
		[enable_sysmocom_bts="yes"],[enable_sysmocom_bts="no"])
AC_MSG_RESULT([$enable_sysmocom_bts])
AM_CONDITIONAL(ENABLE_SYSMOBTS, test "x$enable_sysmocom_bts" = "xyes")

# We share gsm_data.h with OpenBSC and need to be pointed to the source
# directory of OpenBSC for now.
AC_ARG_WITH([openbsc],
	[AS_HELP_STRING([--with-openbsc=INCLUDE_DIR],
		[OpenBSC include directory for openbsc/gsm_data_shared.h])],
	[openbsc_incdir="$withval"],
	[openbsc_incdir="`cd $srcdir; pwd`/../openbsc/openbsc/include"])
AC_SUBST([OPENBSC_INCDIR], $openbsc_incdir)

oldCPPFLAGS=$CPPFLAGS
CPPFLAGS="$CPPFLAGS -I$OPENBSC_INCDIR $LIBOSMOCORE_CFLAGS"
AC_CHECK_HEADER([openbsc/gsm_data_shared.h],[],
	[AC_MSG_ERROR([openbsc/gsm_data_shared.h can not be found in $openbsc_incdir])],
	[])
CPPFLAGS=$oldCPPFLAGS

AC_OUTPUT(
    src/Makefile
    src/common/Makefile
    src/osmo-bts-sysmo/Makefile
    include/Makefile
    include/osmo-bts/Makefile
    tests/Makefile
    tests/paging/Makefile
    tests/agch/Makefile
    tests/cipher/Makefile
    tests/sysmobts/Makefile
    Makefile)
