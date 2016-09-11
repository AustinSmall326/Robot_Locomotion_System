#Variables
CXXFLAGS=-Wall -g -fPIC
CXX = g++
NaoQi_INC = /home/nao/naoqi-sdk-2.1.3.3-linux32/include
NaoQi_LIB = -L/home/nao/naoqi-sdk-2.1.3.3-linux32/lib -lalcommon -lalerror -lalproxies 

#Object Targets
main.o: main.cpp keyframeGen.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC)  main.cpp

keyframeGen.o: keyframeGen.cpp keyframeGen.h $(NaoQi_INC)/alcommon/alproxy.h $(NaoQi_INC)/alcommon/albroker.h $(NaoQi_INC)/alcommon/almodule.h \
                                             $(NaoQi_INC)/alerror/alerror.h $(NaoQi_INC)/alproxies/almemoryproxy.h 
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC) keyframeGen.cpp

#Library Targets
libKeyframeGen.so: main.o keyframeGen.o
	$(CXX) $(CXXFLAGS) -shared -o libKeyframeGen.so main.o keyframeGen.o $(NaoQi_LIB) \
                                                        ../../Representations/COMContainer.o ../../Representations/Point.o ../../Representations/Trajectory.o \
                                                        ../../Representations/Transform.o ../../Kinematics/KinematicsWrapper.o ../../Kinematics/NAOKinematics.o \
    						        ../Iterp/Iterp.o ../PID/PID.o

clean:
	rm *.o
	rm *.so
    
all: keyframeGen.o main.o libKeyframeGen.so
