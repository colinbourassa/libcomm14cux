#!/bin/sh

if [ $# -lt 1 ] ; then
  echo "Usage: $0 <path-to-mxe-environment>"
  exit
fi

MXETYPE="i686-w64-mingw32.shared"
MXE="$1/usr/$MXETYPE"

if [ ! -d "$MXE" ] ; then
  echo "Error: This script currently only supports i686 shared library builds ($MXETYPE)."
  echo "Please update your MXE settings.mk file to include this target type and then build the environment,"
  echo "or change the script to suit your needs."
  exit 1
fi

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE=$MXE/share/cmake/mxe-conf.cmake

