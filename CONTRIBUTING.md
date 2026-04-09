# Contributing to Libbulletjme

Thank you for your interest in contributing to Libbulletjme!

The project isn't seeking financial contributions at this time.
However, there are other ways you can contribute to Libbulletjme:

+ source code additions and improvements
  (to the library, automated test suite, sample applications, or build scripts),
+ additions and improvements to the documentation,
+ bug reports,
+ suggestions,
+ comments on bug reports and suggestions submitted by others, and
+ publicity.

Generally:

+ Anyone may contribute.
+ Contribute only your own work,
  not the work of generative AI or other individuals.
+ No contribution is too small to be considered.

In your communications, please be
as kind, polite, generous, and grateful as you can.
Comparison and criticism should be kept professional and impersonal.


## Pull requests

Contributions of source code and documentation should be submitted
as GitHub pull requests to the relevent repository.
Pull requests should always target the "master" branch.

Each pull request should have a single, specific purpose.
Never combine unrelated changes in a single pull request.

Using static-analysis tools,
it's possible to quickly generate large quantities of changes.
Such changes will be considered on their merits.
In my experience, however, static analysis rarely yields much value.
The most valuable contributions come from people actively using the library,
because users have insight into what's important and what isn't.

Before editing the source code, please read relevant documentation, including
[the implementation page](https://stephengold.github.io/Libbulletjme/lbj-en/English/implementation.html)
and the "code style" section below.

Before investing hours of effort in a pull request,
please communicate with the project manager
to determine whether the changes you envision are actually desired.
Be open to advice on how to implement your idea.

Contributed changes will be reviewed prior to integration.
You can facilitate this process by explicitly requesting review
when you think your pull request is ready.
It's fine to question or discuss review comments.
Multiple review passes may be needed; please be patient.


### Code style

Libbulletjme has an established code style for Java,
which prioritizes readability and ease of maintenance
over creativity and compactness.
C++ source files, on the other hand, follow various styles,
depending where they originated.
Changes and exceptions to the prevailing style are discouraged.

The style of Java sourcecode is loosely based on Google's published rules
and is largely (but not entirely) enforced by Checkstyle.

Project convention (not enforced by Checkstyle) determines the order
of fields and methods within Java source files.
Each file is divided into sections that appear a fixed sequence:

+ license
+ class Javadoc
+ constants
+ fields
+ constructors
+ new methods exposed (including native public methods)
+ new protected methods
+ override methods, grouped by superclass or interface
+ private methods
+ non-public native methods

Within each section, methods are arranged alphanumerically:
by name and then by argument types.
Constants and fields are also arranged alphanumerically within sections:
by type and then by name.


## Conclusions

Thanks for your attention. Your cooperation will be appreciated!
