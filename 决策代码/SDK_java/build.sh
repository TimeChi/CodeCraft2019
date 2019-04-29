#!/bin/bash
basepath=$(cd `dirname $0`; pwd)
APP_HOME=$basepath

#编译
echo building...
MAKE_FILE=$APP_HOME/makelist.txt
libs=-Djava.ext.dirs=$APP_HOME/code/CodeCraft-2019/lib

cd "$APP_HOME/code/CodeCraft-2019/src"
mkdir -p "$APP_HOME/code/CodeCraft-2019/bin"
javac -source 1.8 -target 1.8 -d $APP_HOME/code/CodeCraft-2019/bin -encoding UTF-8 $libs @$MAKE_FILE
tmp=$?
if [ ${tmp} -ne 0 ]
then
 echo "ERROR: javac failed:" ${tmp}
 exit -1
fi



#打包
echo make jar...
cd "$APP_HOME/code/CodeCraft-2019/bin"
JAR_NAME=$APP_HOME/bin/CodeCraft-2019-1.0.jar
jar -cvf $JAR_NAME *
tmp=$?
if [ ${tmp} -ne 0 ]
then
 echo "ERROR: jar failed:" ${tmp}
 exit -1
fi

cd $APP_HOME


if [ -f CodeCraft-2019-1.0.tar.gz ]
then
    rm -f CodeCraft-2019-1.0.tar.gz
fi

mkdir -p $APP_HOME/bin/lib/
cp $APP_HOME/code/CodeCraft-2019/lib/*.jar $APP_HOME/bin/lib/
mkdir -p $APP_HOME/bin/resources/
cp $APP_HOME/code/CodeCraft-2019/src/main/resources/* $APP_HOME/bin/resources/

tar -zcPf CodeCraft-2019-1.0.tar.gz *

echo build jar success!
exit