#!/bin/bash

set -e

S1=~/Git/ext/v-hacd/src/VHACD_Lib
D1=~/Git/Libbulletjme/src/main/native/v-hacd

S2=~/Git/ext/bullet3/src
D2=~/Git/Libbulletjme/src/main/native/bullet3

S3=~/Git/jmonkeyengine/jme3-core/src/main/java
D3=~/Git/Libbulletjme/src/main/java

S4=~/Git/Heart/HeartLibrary/src/main/java
D4=~/Git/Libbulletjme/src/main/java

S5=~/Git/Minie/MinieLibrary/src/main/java
D5=~/Git/Libbulletjme/src/main/java

S6=~/Git/ext/SimMath/src/main/java
D6=~/Git/Libbulletjme/src/main/java

/usr/bin/meld --diff $S3 $D3 --diff $S4 $D4 --diff $S5 $D5 --diff $S6 $D6