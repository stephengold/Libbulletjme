#!/bin/bash

set -e

GitDir=~/NetBeansProjects

S1=$GitDir/ext/v-hacd/src/VHACD_Lib
D1=$GitDir/Libbulletjme/src/main/native/v-hacd

S2=$GitDir/ext/bullet3/src
D2=$GitDir/Libbulletjme/src/main/native/bullet3

S3=$GitDir/jmonkeyengine/jme3-core/src/main/java
D3=$GitDir/Libbulletjme/src/main/java

S4=$GitDir/Heart/HeartLibrary/src/main/java
D4=$GitDir/Libbulletjme/src/main/java

S5=$GitDir/Minie/MinieLibrary/src/main/java
D5=$GitDir/Libbulletjme/src/main/java

S6=$GitDir/ext/SimMath/src/main/java
D6=$GitDir/Libbulletjme/src/main/java

/usr/bin/meld --diff $S1 $D1 --diff $S2 $D2 --diff $S3 $D3 --diff $S4 $D4 --diff $S5 $D5 --diff $S6 $D6