#!/bin/bash
cd /workspace/OpenNi/OpenNI-Linux-x64-2.3
source OpenNIDevEnvironment
cd Samples/ClosestPointViewer
make
cd Bin/x64-Release
./ClosestPointViewer