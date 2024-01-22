<img height="150" src="https://i.imgur.com/YEPFEcx.png" alt="Libbulletjme Project logo">

The [Libbulletjme Project][libbulletjme] adds JNI "glue code"
to portions of [Bullet Physics][bullet]
and [Khaled Mamou's V-HACD Library][vhacd],
enabling 3-D physics simulation from [Java] applications.

Complete source code (in C++ and Java) is provided under
[a mixed open-source license][license].

The project supports the 3 major desktop operating systems:
Windows, Linux, and macOS.  Both the x86 and x86-64 architectures
are supported for each operating system.
It also supports Linux on ARM (armel, armhf, and aarch64) and macOS on ARM
and provides native libraries for the 4 supported Android ABIs
(armeabi-v7a, arm64-v8a, x86, and x86_64),
making a total of 14 platforms.

For each desktop platform, 4 native libraries are distributed:
+ a release build using single-precision arithmetic (the default library)
+ a release build using double-precision arithmetic
+ a debug build using single-precision arithmetic
+ a debug build using double-precision arithmetic

In addition, multithreaded native libraries
are provided for x86_64 architectures running Windows or Linux.

Libbulletjme is used in
[the DynamX Physics Mod for Minecraft](https://dynamx.fr).

Libbulletjme's native libraries are used in [Minie],
which integrates Libbulletjme into [the jMonkeyEngine game engine][jme].
For applications that don't use jMonkeyEngine,
standalone Maven artifacts are provided.


<a name="toc"></a>

## Contents of this document

+ [How to add Libbulletjme to an existing project](#add)
+ [Example applications](#examples)
+ [How to build Libbulletjme from source](#build)
+ [Lexicon of class/enum/struct names](#lexicon)
+ [What's missing](#todo)
+ [External links](#links)
+ [History](#history)
+ [Acknowledgments](#acks)


<a name="add"></a>

## How to add Libbulletjme to an existing project

[How to add Libbulletjme to an existing project](https://stephengold.github.io/Libbulletjme/lbj-en/add.html)

[Jump to table of contents](#toc)


<a name="examples"></a>

## Example applications

+ [HelloLibbulletjme](https://github.com/stephengold/LbjExamples/blob/master/apps/src/main/java/com/github/stephengold/lbjexamples/apps/console/HelloLibbulletjme.java):
  drop a dynamic sphere onto a horizontal surface

+ [HelloVehicle0](https://github.com/stephengold/LbjExamples/blob/master/apps/src/main/java/com/github/stephengold/lbjexamples/apps/console/HelloVehicle0.java):
  drive a vehicle on a horizontal surface

[Jump to table of contents](#toc)


<a name="build"></a>

## How to build Libbulletjme from source

[How to build Libbulletjme from source](https://stephengold.github.io/Libbulletjme/lbj-en/English/build.html)

[Jump to table of contents](#toc)


<a name="lexicon"></a>

## Lexicon of class/enum/struct names

[Lexicon of class/enum/struct names](https://stephengold.github.io/Libbulletjme/lbj-en/English/lexicon.html)

[Jump to table of contents](#toc)


<a name="todo"></a>

## What's missing

[What's missing](https://stephengold.github.io/Libbulletjme/lbj-en/English/overview.html#_whats_missing)

[Jump to table of contents](#toc)


<a name="links"></a>

## External links

+ [The Bullet Physics SDK Manual](https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf)
+ [The Bullet source-code repository](https://github.com/bulletphysics/bullet3) at [GitHub]
+ [The LbjExamples project][examples] at [GitHub]
+ [The Minie project][minie]
+ [The V-HACD Library][vhacd] at [GitHub]
+ [The physics section of the jMonkeyEngine Wiki](https://wiki.jmonkeyengine.org/docs/3.4/physics/physics.html)
+ [The Bullet Forum](https://pybullet.org/Bullet/phpBB3)
+ [The Bullet home page][bullet]
+ [JBullet], a known alternative to Libbulletjme
+ [Alan Chou's game-physics tutorials](http://allenchou.net/game-physics-series/)
+ ["Real-time Vehicle Simulation for Video Games Using the Bullet Physics Library" by Hammad Mazhar](https://sbel.wisc.edu/wp-content/uploads/sites/569/2018/05/Real-time-Vehicle-Simulation-for-Video-Games-Using-the-Bullet-Physics-Library.pdf)
+ ["Vehicle Simulation With Bullet" by Kester Maddock](https://docs.google.com/document/d/18edpOwtGgCwNyvakS78jxMajCuezotCU_0iezcwiFQc)

[Jump to table of contents](#toc)


<a name="history"></a>

## History

The evolution of this project is chronicled in
[its release log][log].

The C++ glue code for Bullet was originally copied from `jme3-bullet-native`,
a library of [jMonkeyEngine][jme].
The soft-body portion was added in 2018,
and is based on the work of Jules (aka "dokthar").

The Java code is based partly jMonkeyEngine,
partly on [Riccardo's V-hacd-java-bindings][vhacdBindings],
and partly on [Minie].
Minie is, in turn, based on `jme3-bullet`, another jMonkeyEngine library.

[Jump to table of contents](#toc)


<a name="acks"></a>

## Acknowledgments

The Libbulletjme Project is derived from open-source software:

  + the [Bullet] physics simulation kit
  + the [jMonkeyEngine][jme] game engine
  + [Dokthar's fork of jMonkeyEngine](https://github.com/dokthar/jmonkeyengine)
  + [Khaled Mamou's V-HACD Library][vhacd] for approximate convex decomposition
  + Riccardo Balbo's [vhacdBindings]
  + Stephen Gold's [Heart] library
  + Paul Speed's [SimMath] library

This project also made use of the following software tools:

  + the [Checkstyle] tool
  + the [FindBugs] source-code analyzer
  + the [GNU Compiler Collection][gcc] and [Project Debugger][gdb]
  + the [Git] revision-control system and GitK commit viewer
  + the [Firefox] and [Google Chrome][chrome] web browsers
  + the [GitKraken] client
  + the [Gradle] build tool
  + the [IntelliJ IDEA][idea] and [NetBeans] integrated development environments
  + the [Java] compiler, standard doclet, and runtime environment
  + [jMonkeyEngine][jme] and the jME3 Software Development Kit
  + the [Linux Mint][mint] operating system
  + the [LLVM Compiler Infrastructure][llvm]
  + the [Markdown] document-conversion tool
  + the [Meld] visual merge tool
  + Microsoft Windows and Visual Studio

I am grateful to Riccardo Balbo (aka "riccardo") for bringing
V-HACD to my attention.

I am grateful to ["dustContributor"](https://github.com/dustContributor)
for [optimizing the cleaner thread](https://github.com/stephengold/Libbulletjme/pull/13).

I am grateful to "elmfrain" for authoring the `GearJoint` class.

I am grateful to [GitHub], [Sonatype], [AppVeyor],
[Travis], [MacStadium], [JFrog], and [Imgur]
for providing free hosting for this project
and many other open-source projects.

<a href="https://www.macstadium.com/opensource/members">
<img height="100" src="https://i.imgur.com/N6J0UBG.png" alt="Powered By MacStadium logo">
</a>

I am grateful to ndebruyn for helping me test the Android native libraries.

I am grateful to Pavly Gerges for helping me test the armhf native library.

I am grateful to Yanis Boudiaf and "qwq" for many helpful suggestions.

I'm also grateful to my dear Holly, for keeping me sane.

If I've misattributed anything or left anyone out, please let me know, so I can
correct the situation: sgold@sonic.net

[Jump to table of contents](#toc)


[appveyor]: https://www.appveyor.com "AppVeyor Continuous Integration"
[bullet]: https://pybullet.org/wordpress "Bullet Real-Time Physics Simulation"
[checkstyle]: https://checkstyle.org "Checkstyle"
[chrome]: https://www.google.com/chrome "Chrome"
[examples]: https://github.com/stephengold/LbjExamples "LbjExamples Project"
[findbugs]: http://findbugs.sourceforge.net "FindBugs Project"
[firefox]: https://www.mozilla.org/en-US/firefox "Firefox"
[gcc]: https://gcc.gnu.org "GNU Compiler Collection"
[gdb]: https://www.gnu.org/software/gdb/ "GNU Project Debugger"
[git]: https://git-scm.com "Git"
[github]: https://github.com "GitHub"
[gitkraken]: https://www.gitkraken.com "GitKraken client"
[gradle]: https://gradle.org "Gradle Project"
[heart]: https://github.com/stephengold/Heart "Heart Project"
[idea]: https://www.jetbrains.com/idea/ "IntelliJ IDEA"
[imgur]: https://imgur.com/ "Imgur"
[java]: https://java.com "Java"
[jbullet]: http://jbullet.advel.cz "JBullet"
[jfrog]: https://www.jfrog.com "JFrog"
[jme]: https://jmonkeyengine.org  "jMonkeyEngine Project"
[libbulletjme]: https://stephengold.github.io/Libbulletjme "Libbulletjme Project"
[license]: https://github.com/stephengold/Libbulletjme/blob/master/LICENSE "Libbulletjme license"
[llvm]: https://www.llvm.org "LLVM Compiler"
[log]: https://github.com/stephengold/Libbulletjme/blob/master/release-notes.md "release log"
[macstadium]: https://www.macstadium.com/ "MacStadium"
[markdown]: https://daringfireball.net/projects/markdown "Markdown Project"
[meld]: https://meldmerge.org "Meld merge tool"
[minie]: https://stephengold.github.io/Minie "Minie Project"
[mint]: https://linuxmint.com "Linux Mint Project"
[netbeans]: https://netbeans.org "NetBeans Project"
[simmath]: https://github.com/Simsilica/SimMath "SimMath Library"
[sonatype]: https://www.sonatype.com "Sonatype"
[travis]: https://travis-ci.com "Travis CI"
[vhacd]: https://github.com/kmammou/v-hacd "V-HACD Library"
[vhacdBindings]: https://github.com/riccardobl/v-hacd-java-bindings "Riccardo's V-hacd-java-bindings Project"