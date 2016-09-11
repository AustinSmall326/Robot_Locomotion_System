# Variables
CXXFLAGS=-Wall -g
CXX = g++

# Object Targets
TestKinematics.o: TestKinematics.cpp TestKinematics.h
	$(CXX) $(CXXFLAGS) -c TestKinematics.cpp

KinematicsWrapper.o: KinematicsWrapper.cpp KinematicsWrapper.h
	$(CXX) $(CXXFLAGS) -c KinematicsWrapper.cpp

NAOKinematics.o:  NAOKinematics.cpp NAOKinematics.h
	$(CXX) $(CXXFLAGS) -c NAOKinematics.cpp

clean:
	rm -f *.o

all: NAOKinematics.o KinematicsWrapper.o TestKinematics.o
