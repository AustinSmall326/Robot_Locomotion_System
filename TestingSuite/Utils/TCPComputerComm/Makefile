#Variables
CXXFLAGS=-Wall -g -fPIC 
CXX = g++
BOOST_INCLUDE = -I/usr/local/include
BOOST_LIB = -L/usr/local/lib -lboost_thread -lboost_system

SHELL:=/bin/bash

#Object Targets
TCPHeartbeatWrapper.o: TCPHeartbeatWrapper.cpp TCPHeartbeatWrapper.h ../TCPLibrary/TCPStream.h ../TCPLibrary/TCPConnector.h ../TCPLibrary/TCPAcceptor.h
	$(CXX) $(CXXFLAGS) -c TCPHeartbeatWrapper.cpp

clean:
	rm -f *.o
	rm -f *.mex*
    
#mex: TCPHeartbeatWrapper.o ../TCPLibrary/TCPStream.o ../TCPLibrary/TCPConnector.o ../TCPLibrary/TCPAcceptor.o
#	mex TCPComm.cpp $(BOOST_INCLUDE) $(BOOST_LIB) \
#		        TCPHeartbeatWrapper.o ../TCPLibrary/TCPStream.o ../TCPLibrary/TCPConnector.o ../TCPLibrary/TCPAcceptor.o

all: TCPHeartbeatWrapper.o
	  
