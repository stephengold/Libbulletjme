#! /bin/bash
# Copy the built MacOSX_ARM64 natives to $1 and rename them for distribution.

set -euo pipefail

if [[ $(uname) != 'Darwin' ]]; then
    echo "Skipped because kernel=$(uname)" >&2
    exit 2
fi

SRC=~/Git/Libbulletjme/build/libs/bulletjme/shared
if [[ "$#" -ne 1 ]]; then
    DST=~/Git/Libbulletjme
else
    DST=$1
fi

cp $SRC/debug/dp/libbulletjme.dylib $DST/MacOSX_ARM64DebugDp_libbulletjme.dylib
cp $SRC/debug/sp/libbulletjme.dylib $DST/MacOSX_ARM64DebugSp_libbulletjme.dylib
cp $SRC/release/dp/libbulletjme.dylib $DST/MacOSX_ARM64ReleaseDp_libbulletjme.dylib
cp $SRC/release/sp/libbulletjme.dylib $DST/MacOSX_ARM64ReleaseSp_libbulletjme.dylib

echo "Copied 4 built natives to $DST"
echo " and renamed them for distribution."