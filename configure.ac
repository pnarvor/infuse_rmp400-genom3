dnl Autoconf file for building rmp400 codels library.

AC_PREREQ(2.59)

AC_INIT([rmp400-genom3],[0.1.99],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acrmp400.h])
AM_INIT_AUTOMAKE([foreign no-define])

dnl Compilers
dnl
LT_INIT([disable-static])
AC_PROG_CC
AC_PROG_CXX


dnl External packages
PKG_CHECK_MODULES(requires, [
  genom3 >= 2.99.4
  gyro-libs >= 3.0.99.0
  openrobots-idl >= 1.1
  rmp-libs >= 0.4.99, gyro-libs, felib
])

AC_SEARCH_LIBS([clock_gettime], [rt])

AC_PATH_PROG(GENOM3, [genom3], AC_MSG_ERROR([genom3 tool not found], 2))


dnl --with-templates option
AG_OPT_TEMPLATES([$GENOM3 ],
    [$srcdir/rmp400.gen])

dnl Doc
AM_MISSING_PROG([ASCIIDOCTOR], [asciidoctor])

dnl Output
AC_CONFIG_FILES([
	rmp400-genom3.pc
	rmp400-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES
