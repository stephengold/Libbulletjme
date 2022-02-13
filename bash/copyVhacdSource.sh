#!/bin/bash

# Copy V-HACD source files used by Libbulletjme
# from the SRC directory to the DST directory.

set -e

GitDir=~/NetBeansProjects

SRC=$GitDir/ext/v-hacd/src/VHACD_Lib
DST=$GitDir/Libbulletjme/src/main/native/v-hacd

cd $SRC
cp --recursive inc public src $DST

cd $DST
find . -name 'bt*' -exec rm {} \;

git status --short

cd $SRC
pwd
git log -1 --format="updated V-HACD source to %d (SHA1 ID=%h) %s"
