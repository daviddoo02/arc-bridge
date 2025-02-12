#!/bin/sh

LCM_GEN=robot_lcm
# Try to automatically determine where the LCM java file is
LCM_JAR=`pkg-config --variable=classpath lcm-java`
if [ $? != 0 ] ; then
    if [ -e /usr/local/share/java/lcm.jar ] ; then
        LCM_JAR=/usr/local/share/java/lcm.jar
    else
        if [ -e ../lcm/lcm-java/lcm.jar ] ; then
            LCM_JAR=../lcm/lcm-java/lcm.jar
        else
            LCM_JAR=./lcm.jar
        fi
    fi
fi
# Remove old generated types
rm -rf $LCM_GEN
# Make a folder to store generated types if not exists
mkdir -p $LCM_GEN
# Generate python types
lcm-gen -p ./*.lcm
# Generate java types
lcm-gen -j ./*.lcm
# Generate C++ types
lcm-gen -x ./*.lcm
# Compile java types
# java release 8 is used for compatability of MATLAB >= 2023b
javac -cp $LCM_JAR $LCM_GEN/*.java --release 8 
jar cf robot_types.jar $LCM_GEN/*.class
echo "Successfully generated LCM types"
