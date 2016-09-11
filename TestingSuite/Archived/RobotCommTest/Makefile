#Variables
CXXFLAGS=-Wall -g -fPIC
CXX = g++
NaoQi_INC = /home/nao/naoqi-sdk-2.1.3.3-linux32/include
NaoQi_LIB = -L/home/nao/naoqi-sdk-2.1.3.3-linux32/lib -lalcommon -lalerror

#Object Targets
main.o: main.cpp RobotCommTest.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC)  main.cpp

RobotCommTest.o: RobotCommTest.cpp RobotCommTest.h $(NaoQi_INC)/alcommon/alproxy.h $(NaoQi_INC)/alcommon/albroker.h $(NaoQi_INC)/alcommon/almodule.h \
                     $(NaoQi_INC)/alerror/alerror.h 
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC) RobotCommTest.cpp

#Library Targets
libRobotCommTest.so: main.o RobotCommTest.o
	$(CXX) $(CXXFLAGS) -shared -o libRobotCommTest.so main.o RobotCommTest.o $(NaoQi_LIB)

clean:
	rm *.o
	rm *.so
    
all: RobotCommTest.o main.o libRobotCommTest.so
