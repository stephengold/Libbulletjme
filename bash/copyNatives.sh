#! /bin/bash
set -euo pipefail

SRC=~/Git/Libbulletjme/build/libs/bulletjme/shared
DST=~/Git/Libbulletjme

cp $SRC/debug/dp/libbulletjme.dylib $DST/MacOSX_ARM64DebugDp_libbulletjme.dylib
cp $SRC/debug/sp/libbulletjme.dylib $DST/MacOSX_ARM64DebugSp_libbulletjme.dylib
cp $SRC/release/dp/libbulletjme.dylib $DST/MacOSX_ARM64ReleaseDp_libbulletjme.dylib
cp $SRC/release/sp/libbulletjme.dylib $DST/MacOSX_ARM64ReleaseSp_libbulletjme.dylib
