#Variables
CXXFLAGS=-Wall -g 
CXX = g++
NaoQi_INC = /home/nao/naoqi-sdk-2.1.3.3-linux32/include
NaoQi_LIB = -L/home/nao/naoqi-sdk-2.1.3.3-linux32/lib -lalcommon

#Object Targets

Iterp.o: Iterp.cpp Iterp.h 
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC) Iterp.cpp

clean:
	rm -f *.o
    
all: Iterp.o
