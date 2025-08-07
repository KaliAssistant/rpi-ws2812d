# Initialize Autoconf and define the project name, version, and bug report address.
AC_INIT([rpi-ws2812d],[0.0.1],[work.kaliassistant.github@gmail.com])
AM_INIT_AUTOMAKE([foreign -Wall -Werror])
AC_CONFIG_SRCDIR([src/ws2812d.c])
AC_CONFIG_HEADERS([config.h])

# Check for a C compiler.
AC_PROG_CC
LT_INIT

# prevent libtool from adding -rpath
hardcode_libdir_flag_spec=
runpath_var=

# Check for the math library 'm'.
AC_SEARCH_LIBS([cos], [m])

# Checks for typedefs, structures, and compiler characteristics.
AC_C_BIGENDIAN

# Define version macros to be used in C code.
AC_DEFINE_UNQUOTED([VERSION_MAJOR], [0], [Major version number])
AC_DEFINE_UNQUOTED([VERSION_MINOR], [0], [Minor version number])
AC_DEFINE_UNQUOTED([VERSION_MICRO], [1], [Micro version number])

AC_ARG_WITH([rpi-ws281x],
    [AS_HELP_STRING([--with-rpi-ws281x=PATH], [Path to rpi_ws281x installation])],
    [RPIWS281X_PATH="$withval"],
    [RPIWS281X_PATH=""])

if test -n "$RPIWS281X_PATH"; then
    CPPFLAGS="$CPPFLAGS -I$RPIWS281X_PATH/include"
    LDFLAGS="$LDFLAGS -L$RPIWS281X_PATH/lib"
fi

AC_CHECK_LIB([ws2811], [ws2811_init],
    [AC_DEFINE([HAVE_WS281X], [1], [Define if you have ws281x])],
    [AC_MSG_ERROR([libws2811 not found. Use --with-rpi-ws281x or install it.])])

# Create the output files.
AC_CONFIG_FILES([Makefile src/Makefile])
AC_OUTPUT
