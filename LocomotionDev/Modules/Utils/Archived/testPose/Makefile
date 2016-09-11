#Variables
CXXFLAGS=-Wall -g -fPIC -MMD
CXX = g++
NaoQi_INC = /home/nao/naoqi-sdk-2.1.3.3-linux32/include
NaoQi_LIB = -L/home/nao/naoqi-sdk-2.1.3.3-linux32/lib -lalcommon -lalerror

#Object Targets
main.o: main.cpp testPose.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC)  main.cpp

testPose.o: testPose.cpp testPose.h $(NaoQi_INC)/alcommon/alproxy.h $(NaoQi_INC)/alcommon/albroker.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC) testPose.cpp

#Library Targets
libtestPose.so: main.o testPose.o
	$(CXX) $(CXXFLAGS) -shared -o libtestPose.so ../../Representations/Transform.o \
                                      ../../Kinematics/KinematicsWrapper.o ../../Kinematics/NAOKinematics.o \
                                      main.o testPose.o $(NaoQi_LIB)

clean:
	rm *.o 
    
all: testPose.o main.o libtestPose.so
