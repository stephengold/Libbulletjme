#! /bin/bash
set -euo pipefail

if [[ "$#" -ne 1 ]]; then
    echo "Missing parameter:  version" >&2
    exit 2
fi

cd ~/Releases/Libbulletjme/$1

grep signing.password ~/.gradle/gradle.properties
read -p "Press the [Enter] key to start signing `pwd`"

chmod 755 .

for f in *.{jar,module,pom};
do
  if [ ! -f $f.asc ]; then
    /usr/bin/gpg2 -ab $f ;
  fi
done;

chmod 444 *
chmod 555 .

ls -l
