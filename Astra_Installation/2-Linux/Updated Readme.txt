#############Note###################
# For user with ARM based development board:
# With CPU Structure older than Cortex A17, use OpenNI-Linux-Arm-2.3 Nofilter.tar for better performance
####################################


# There are two zip files, one is for 32bit machine, the other one is for 64bit

# We choose 64bit(x64) and make the example as follows:


# To run visual samples(e.g., SimpleViewer), you will need freeglut3 header and libaries, please install:

$ sudo apt-get install build-essential freeglut3 freeglut3-dev

#check udev version, Orbbec Driver need libudev.so.1, if can't find it, can make symbolic link from libudev.so.x.x,
#which usually locate in /lib/x86_64-linux-gnu or /lib/i386-linux-gnu
$ldconfig -p | grep libudev.so.1
$cd /lib/x86_64-linux-gnu
$sudo ln -s libudev.so.x.x.x libudev.so.1

# copy tgz file to any place you want(e.g., Home)

# unzip tgz file
$ tar zxvf OpenNI-Linux-x64-2.2-0118.tgz
$ cd OpenNI-Linux-x64-2.2

# run install.sh to generate OpenNIDevEnvironment, which contains OpenNI development environment 
#(sudo chmod a+x install.sh)

$ sudo ./install.sh

# please replug in the device for usb-register

# add environment variables
$ source OpenNIDevEnvironment

# build sample(e.g., SimpleViewer)
$ cd Samples/SimpleViewer
$ make

# run sample
# connect sensor
$ cd Bin/x64-Release
$ ./SimpleViewer

# now you should be able to see a GUI window showing the depth stream video

# If the Debian Jessie Lite is used for testing, it may require the following installation for properly start the viewer.

$ sudo apt-get install libgl1-mesa-dri

