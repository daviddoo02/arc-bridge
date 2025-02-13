#!/bin/bash

# Detect OS type
OS_TYPE=$(uname -s)
if [[ "$OS_TYPE" == "Darwin" ]]; then
    SYSTEM="macOS"
    PREF_DIR_BASE=~/Library/Application\ Support/MathWorks/MATLAB
elif [[ "$OS_TYPE" == "Linux" ]]; then
    SYSTEM="Linux"
    PREF_DIR_BASE=~/.matlab
else
    echo "Unsupported OS: $OS_TYPE"
    exit 1
fi

echo "Detected OS: $SYSTEM"

CURRENT_DIR=$(pwd)
LCM_GEN_DIR=robot_lcm
LCM_TYPE_JAR=$CURRENT_DIR/robot_types.jar

# Try to automatically determine where the LCM java file is
LCM_JAR=`pkg-config --variable=classpath lcm-java`
if [ $? != 0 ] ; then
    if [ -e /usr/local/share/java/lcm.jar ] ; then
        LCM_JAR=/usr/local/share/java/lcm.jar
    else
        if [ -e $CURRENT_DIR/lcm/lcm-java/lcm.jar ] ; then
            LCM_JAR=$CURRENT_DIR/lcm/lcm-java/lcm.jar
        else
            LCM_JAR=$CURRENT_DIR/lcm.jar
        fi
    fi
fi
echo "Found LCM jar file: $LCM_JAR"

# Remove old generated types
rm -rf $LCM_GEN_DIR
# Make a folder to store generated types if not exists
mkdir -p $LCM_GEN_DIR
# Generate python types
lcm-gen -p ./*.lcm
# Generate java types
lcm-gen -j ./*.lcm
# Generate C++ types
lcm-gen -x ./*.lcm
# Compile java types
# java release 8 is used for compatability of MATLAB >= 2023b
javac -cp $LCM_JAR $LCM_GEN_DIR/*.java --release 8 
jar cf $LCM_TYPE_JAR $LCM_GEN_DIR/*.class
echo "Successfully generated LCM types"

# Find the latest MATLAB preference folder
if [[ -d "$PREF_DIR_BASE" ]]; then
    LATEST_PREF_DIR=$(ls -d "$PREF_DIR_BASE"/R* 2>/dev/null | sort -V | tail -n 1)
    
    if [[ -z "$LATEST_PREF_DIR" ]]; then
        echo "No MATLAB preference folder found."
        exit 1
    fi

    echo "Using MATLAB preference folder: $LATEST_PREF_DIR"
else
    echo "MATLAB preference base directory not found: $PREF_DIR_BASE"
    exit 1
fi

# Write to javaclasspath.txt
JAVACLASS_PATH_FILE="$LATEST_PREF_DIR/javaclasspath.txt"

# Add the generated LCM types to javaclasspath.txt
echo "Adding generated LCM types to $JAVACLASS_PATH_FILE"
echo -e "$LCM_JAR\n$LCM_TYPE_JAR\n" > "$JAVACLASS_PATH_FILE"
