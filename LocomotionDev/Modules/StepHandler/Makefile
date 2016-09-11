#Variables
CXXFLAGS=-Wall -g -fPIC
CXX = g++
NaoQi_INC = /home/nao/naoqi-sdk-2.1.3.3-linux32/include
NaoQi_LIB = -L/home/nao/naoqi-sdk-2.1.3.3-linux32/lib -lalcommon -lalerror -lalproxies

#Object Targets
main.o: main.cpp StepHandler.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC)  main.cpp

StepHandler.o: StepHandler.cpp StepHandler.h 
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC) StepHandler.cpp

#Library Targets
libStepHandler.so: main.o StepHandler.o
	$(CXX) $(CXXFLAGS) -shared -o ../../../Libs/libStepHandler.so main.o StepHandler.o $(NaoQi_LIB) \
				      ../Representations/COMContainer.o ../Representations/COMContainerTests.o ../Representations/Point.o ../Representations/PointTests.o \
                                      ../Representations/Trajectory.o ../Representations/TrajectoryTests.o ../Representations/Transform.o \
				      ../Representations/TransformTests.o ../Kinematics/KinematicsWrapper.o ../Kinematics/NAOKinematics.o ../Kinematics/TestKinematics.o \
				      ../Utils/Iterp/Iterp.o

clean:
	rm -f *.o
	rm -f ../../../Libs/libStepHandler.so 
    
all: StepHandler.o main.o libStepHandler.so
