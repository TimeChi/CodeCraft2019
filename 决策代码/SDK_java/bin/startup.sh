#!/bin/bash
basepath=$(cd `dirname $0`; pwd)
APP_HOME=$basepath/..

JAVA=$JAVA_HOME/bin/java
commons_logging_lib=$APP_HOME/code/CodeCraft-2019/lib/commons-logging-1.2.jar
log4j_lib=$APP_HOME/code/CodeCraft-2019/lib/log4j-1.2.17.jar

JVM_OPT="-Xms64M -Xmx64M"
JVM_OPT="$JVM_OPT -Djava.ext.dirs=./lib"
JVM_OPT="$JVM_OPT -Djava.library.path=$APP_HOME/bin"
JVM_OPT="$JVM_OPT -classpath"
JVM_OPT="$JVM_OPT $APP_HOME/bin/CodeCraft-2019-1.0.jar:$APP_HOME/bin/resources/"
carPath=$1
roadPath=$2
crossPath=$3
presetAnswer=$4
answerPath=$5
echo "$JAVA $JVM_OPT com.huawei.Main $carPath $roadPath $crossPath $answerPath 2>&1"
$JAVA  $JVM_OPT com.huawei.Main $carPath $roadPath $crossPath $presetAnswer $answerPath 2>&1
exit