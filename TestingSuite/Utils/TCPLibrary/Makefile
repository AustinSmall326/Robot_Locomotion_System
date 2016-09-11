#Variables
CXXFLAGS=-Wall -g -fPIC
CXX = g++

#Object Targets
TCPStream.o: TCPStream.cpp TCPStream.h
	$(CXX) $(CXXFLAGS) -c TCPStream.cpp

TCPAcceptor.o: TCPAcceptor.cpp TCPAcceptor.h TCPStream.h 
	$(CXX) $(CXXFLAGS) -c TCPAcceptor.cpp

TCPConnector.o: TCPConnector.cpp TCPConnector.h TCPStream.h
	$(CXX) $(CXXFLAGS) -c TCPConnector.cpp

clean:
	rm -f *.o
    
all: TCPStream.o TCPAcceptor.o TCPConnector.o 
