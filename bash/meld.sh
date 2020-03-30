#!/bin/bash

set -e

S1=/home/sgold/Git/ext/v-hacd/src/VHACD_Lib
D1=/home/sgold/Git/Libbulletjme/src/main/native/v-hacd

S2=/home/sgold/Git/ext/bullet3/src
D2=/home/sgold/Git/Libbulletjme/src/main/native/bullet3

S3=/home/sgold/Git/jmonkeyengine/jme3-core/src/main/java
D3=/home/sgold/Git/Libbulletjme/src/main/java

S4=/home/sgold/Git/Heart/HeartLibrary/src/main/java
D4=/home/sgold/Git/Libbulletjme/src/main/java

S5=/home/sgold/Git/Minie/MinieLibrary/src/main/java
D5=/home/sgold/Git/Libbulletjme/src/main/java

/usr/bin/meld --diff $S3 $D3 --diff $S4 $D4 --diff $S5 $D5