#!/bin/bash

# Copy Bullet source files used by Libbulletjme
# from the SRC directory to the DST directory.

set -e

GitDir=~/NetBeansProjects

SRC=$GitDir/ext/bullet3/src
DST=$GitDir/Libbulletjme/src/main/native/bullet3

cd $SRC
cp --recursive btBulletCollisionCommon.h btBulletDynamicsCommon.h \
         BulletCollision BulletDynamics BulletSoftBody LinearMath \
         $DST

cd $DST
find . -name CMakeLists.txt -exec rm {} \;
find . -name premake4.lua -exec rm {} \;
rm --recursive LinearMath/TaskScheduler

git status --short

cd $SRC
pwd
git log -1 --format="updated Bullet source to %d (SHA1 ID=%h) %s"
