AC_INIT([removelogo], [1], [https://github.com/Noctem/vapoursynth-removelogos/issues], [https://github.com/Noctem/vapoursynth-removelogo/])

AM_INIT_AUTOMAKE([foreign no-dist-gzip dist-xz subdir-objects no-define])
AM_SILENT_RULES([yes])

LT_INIT([disable-static win32-dll])

AC_PROG_CC

AC_CANONICAL_HOST

PKG_CHECK_MODULES([LIBAV], [libavcodec libavformat libavutil libswscale])
PKG_CHECK_MODULES([VAPOURSYNTH], [vapoursynth])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
