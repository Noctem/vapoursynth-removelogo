warning_flags = -Wall -Wextra -Wno-unused-parameter -Wshadow
common_cflags = -O3 $(warning_flags)
AM_CFLAGS = $(common_cflags) $(VAPOURSYNTH_CFLAGS)

lib_LTLIBRARIES = libremovelogo.la

libremovelogo_la_SOURCES = src/removelogo.c

libremovelogo_la_LDFLAGS = -no-undefined -avoid-version $(PLUGINLDFLAGS)
