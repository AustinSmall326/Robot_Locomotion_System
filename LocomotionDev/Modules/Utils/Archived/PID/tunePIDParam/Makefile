#Variables
CXXFLAGS=-Wall -g -fPIC
CXX = g++
NaoQi_INC = /home/nao/naoqi-sdk-2.1.3.3-linux32/include
NaoQi_LIB = -L/home/nao/naoqi-sdk-2.1.3.3-linux32/lib -lalcommon -lalerror -lalproxies 

#Object Targets
main.o: main.cpp tunePIDParam.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC)  main.cpp

tunePIDParam.o: tunePIDParam.cpp tunePIDParam.h $(NaoQi_INC)/alcommon/alproxy.h $(NaoQi_INC)/alcommon/albroker.h $(NaoQi_INC)/alcommon/almodule.h \
                                             $(NaoQi_INC)/alerror/alerror.h $(NaoQi_INC)/alproxies/almemoryproxy.h 
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC) tunePIDParam.cpp

#Library Targets
libTunePIDParam.so: main.o tunePIDParam.o
	$(CXX) $(CXXFLAGS) -shared -o libTunePIDParam.so main.o tunePIDParam.o $(NaoQi_LIB) ../PID.o

clean:
	rm *.o
	rm *.so
    
all: tunePIDParam.o main.o libTunePIDParam.so
