#!/bin/bash

# Copy JNI header files used by Libbulletjme
# from the SRC directory to the DST directory.

set -e

SRC=/home/sgold/Git/Minie/MinieLibrary/build/cpp
DST=/home/sgold/Git/Libbulletjme/src/main/native/glue

cd $SRC
cp com_jme3_bullet_*.h $DST
cp vhacd_VHACD*.h $DST

cd $DST
git status --short
