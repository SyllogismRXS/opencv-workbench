#!/bin/bash
###################################################################
# Title       : DEPENDENCIES.sh
# Description : Installs libraries listed in DEPENDENCIES array 
# Supports    : Ubuntu, Red Hat, CentOS
###################################################################

usage()
{
cat << EOF
usage: sudo $0 <linux-variant>
This script installs all dependencies for opencv-workbench

EOF
}

###################################################################
# Dependencies array.  
# Add new dependencies here.
###################################################################
echo "---------------------------------------------------"
echo "Detecting Linux operating system variant."

# Dependencies for all Linux variants...
DEPS_COMMON=(
qt-sdk
cmake
mercurial
libboost1.46-all-dev
git
libv4l-dev
build-essential 
checkinstall 
libfaac-dev 
libjack-jackd2-dev 
libmp3lame-dev 
libopencore-amrnb-dev 
libopencore-amrwb-dev 
libsdl1.2-dev 
libtheora-dev 
libva-dev 
libvdpau-dev 
libvorbis-dev 
libx11-dev 
libxfixes-dev 
libxvidcore-dev 
texi2html 
yasm 
zlib1g-dev
libgstreamer0.10-0 
libgstreamer0.10-dev 
gstreamer0.10-tools 
gstreamer0.10-plugins-base 
libgstreamer-plugins-base0.10-dev 
gstreamer0.10-plugins-good 
gstreamer0.10-plugins-ugly 
gstreamer0.10-plugins-bad 
gstreamer0.10-ffmpeg
libgtk2.0-0 
libgtk2.0-dev
libjpeg62
libjpeg62-dev
)

# Dependencies only for RedHat, CentOS, etc.
DEPS_RPM=(
)

# Dependencies only for Ubuntu
DEPS_DPKG=(
)

# Dependencies only for ArchLinux
DEPS_PACMAN=()

#Require the script to be run as root
if [[ $(/usr/bin/id -u) -ne 0 ]]; then
    echo "This script must be run as root because libraries will be installed."
    usage
    exit
fi

###################################################################
# Determine which package system is being used
# This helps determine which version of linux is being used
###################################################################
# Ubuntu
if which apt-get &> /dev/null; then
    DEPENDENCIES=("${DEPS_COMMON[@]}" "${DEPS_DPKG[@]}")
    echo "This is Ubuntu. Using dpkg."

# OpenSuse, Mandriva, Fedora, CentOs, ecc. (with rpm)
elif which rpm &> /dev/null; then
    DEPENDENCIES=("${DEPS_COMMON[@]}" "${DEPS_RPM[@]}")
    echo "This is Red Hat / CentOS. Using rpm."

# ArchLinux (with pacman)
elif which pacman &> /dev/null; then
    DEPENDENCIES=("${DEPS_COMMON[@]}" "${DEPS_PACMAN}")
    echo "This is ArchLinux. Using pacman."
else
    echo "Can't determine operating system or package system."
    exit
fi

###################################################################
# Determine which packages are missing
###################################################################
echo "Detecting which required packages are not installed."

dep_len=${#DEPENDENCIES[@]}

PKGSTOINSTALL=""
for (( i=0; i < $dep_len; i++))
do
    if which apt-get &> /dev/null; then
	if [[ ! `dpkg -l | grep -w "ii  ${DEPENDENCIES[$i]} "` ]]; then
	    PKGSTOINSTALL=$PKGSTOINSTALL" "${DEPENDENCIES[$i]}
	fi
	# OpenSuse, Mandriva, Fedora, CentOs, ecc. (with rpm)
    elif which rpm &> /dev/null; then
	if [[ `rpm -q ${DEPENDENCIES[$i]}` == "package "${DEPENDENCIES[$i]}" is not installed" ]]; then 
	    PKGSTOINSTALL=$PKGSTOINSTALL" "${DEPENDENCIES[$i]}
	fi
	# ArchLinux (with pacman)
    elif which pacman &> /dev/null; then
	if [[ ! `pacman -Qqe | grep "${DEPENDENCIES[$i]}"` ]]; then
	    PKGSTOINSTALL=$PKGSTOINSTALL" "${DEPENDENCIES[$i]}
	fi
    else
	# If it's impossible to determine if there are missing dependencies, mark all as missing
	PKGSTOINSTALL=$PKGSTOINSTALL" "${DEPENDENCIES[$i]}
    fi
done

###################################################################
# Install missing dependencies.
# First, ask user.
###################################################################
if [ "$PKGSTOINSTALL" != "" ]; then
    echo "The following dependencies are missing:" 
    echo "${PKGSTOINSTALL}"
    echo -n "Want to install them? (Y/n): "
    read SURE
# If user want to install missing dependencies
    if [[ $SURE = "Y" || $SURE = "y" || $SURE = "" ]]; then
    # Debian, Ubuntu and derivatives (with apt-get)
	if which apt-get &> /dev/null; then
	    apt-get install $PKGSTOINSTALL
	# OpenSuse (with zypper)
	elif which zypper &> /dev/null; then
	    zypper in $PKGSTOINSTALL
	# Mandriva (with urpmi)
	elif which urpmi &> /dev/null; then
	    urpmi $PKGSTOINSTALL
	# Fedora and CentOS (with yum)
	elif which yum &> /dev/null; then
	    echo "yum install $PKGSTOINSTALL"
	    yum install $PKGSTOINSTALL
	# ArchLinux (with pacman)
	elif which pacman &> /dev/null; then
	    pacman -Sy $PKGSTOINSTALL
	# Else, if no package manager has been founded
	else
	# Set $NOPKGMANAGER
	    NOPKGMANAGER=TRUE
	    echo "ERROR: impossible to found a package manager in your sistem. Please, install manually ${DEPENDENCIES[*]}."
	fi

        # Check if installation is successful
	if [[ $? -eq 0 && ! $NOPKGMANAGER == "TRUE" ]] ; then
	    echo "All dependencies are satisfied."
        else
	    # Else, if installation isn't successful
	    echo "ERROR: impossible to install some missing dependencies. Please, install manually ${DEPENDENCIES[*]}."
	fi

    else
	# Else, if user don't want to install missing dependencies
	echo "WARNING: Some dependencies may be missing. So, please, install manually ${DEPENDENCIES[*]}."
    fi
else
    echo "All dependencies are installed. No further action is required."
fi
