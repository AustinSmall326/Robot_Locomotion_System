#Variables
CXXFLAGS=-Wall -g -fPIC
CXX = g++
NaoQi_INC = /home/nao/naoqi-sdk-2.1.3.3-linux32/include
NaoQi_LIB = -L/home/nao/naoqi-sdk-2.1.3.3-linux32/lib -lalcommon -lalerror -lalproxies
TCP_LIB   = ../TCPLibrary

#Object Targets
main.o: main.cpp TCPRobotComm.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC)  main.cpp

TCPRobotComm.o: TCPRobotComm.cpp TCPRobotComm.h $(TCP_LIB)/TCPStream.h $(TCP_LIB)/TCPAcceptor.h $(TCP_LIB)/TCPConnector.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC) -I$(TCP_LIB) TCPRobotComm.cpp

#Library Targets
libTCPRobotComm.so: main.o TCPRobotComm.o
	$(CXX) $(CXXFLAGS) -shared -o ../../../Libs/libTCPRobotComm.so main.o TCPRobotComm.o $(NaoQi_LIB) $(TCP_LIB)/TCPStream.o \
                                      $(TCP_LIB)/TCPAcceptor.o $(TCP_LIB)/TCPConnector.o

clean:
	rm -f *.o
	rm -f ../../../Libs/libTCPRobotComm.so 
    
all: TCPRobotComm.o main.o libTCPRobotComm.so
