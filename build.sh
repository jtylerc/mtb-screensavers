#!/bin/sh
# Builds every screensaver under screensavers/ into build/<Name>.scr, with llvm-mingw
# (https://github.com/mstorsjo/llvm-mingw) from Linux, WSL or Windows. Put the toolchain's bin directory on
# PATH, or point LLVM_MINGW at the toolchain's root directory. Pass names to build only some: ./build.sh mtb-cad
# Each .scr is named after the name Windows shows for it (string 1 in its resources.rc), spaces as hyphens.
set -e
cd "$(dirname "$0")"
BIN=${LLVM_MINGW:+$LLVM_MINGW/bin/}
CXX=${BIN}x86_64-w64-mingw32-clang++
RC=${BIN}x86_64-w64-mingw32-windres
mkdir -p build
for dir in ${*:-screensavers/*}; do
  dir=screensavers/$(basename "$dir")
  name=$(basename "$dir")
  out=$(sed -n 's/^STRINGTABLE { 1, "\(.*\)" }/\1/p' "$dir/resources.rc" | tr ' ' '-')   # "MTB CAD" -> MTB-CAD.scr
  [ -n "$out" ] || out=$name
  $RC "$dir/resources.rc" -O coff -o "build/$name-resources.o"
  $CXX -std=c++20 -O2 -municode -mwindows -static -Wall -Wno-missing-braces -Icommon \
    common/*.cpp "$dir"/*.cpp "build/$name-resources.o" \
    -ld2d1 -ldwrite -ldwmapi -lole32 -lshell32 -luser32 -lgdi32 -luuid -s -o "build/$out.scr"
  echo "built build/$out.scr"
done
