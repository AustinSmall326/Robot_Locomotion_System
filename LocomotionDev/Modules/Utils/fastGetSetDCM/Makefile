#Variables
CXXFLAGS=-Wall -g -fPIC
CXX = g++
NaoQi_INC = /home/nao/naoqi-sdk-2.1.3.3-linux32/include
NaoQi_LIB = -L/home/nao/naoqi-sdk-2.1.3.3-linux32/lib -lalcommon -lalerror -lalproxies \
                                                      -lalmemoryfastaccess

#Object Targets
main.o: main.cpp fastGetSetDCM.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC)  main.cpp

fastGetSetDCM.o: fastGetSetDCM.cpp fastGetSetDCM.h $(NaoQi_INC)/alcommon/alproxy.h $(NaoQi_INC)/alcommon/albroker.h \
                     $(NaoQi_INC)/alcommon/almodule.h $(NaoQi_INC)/alerror/alerror.h $(NaoQi_INC)/alproxies/dcmproxy.h \
                     $(NaoQi_INC)/almemoryfastaccess/almemoryfastaccess.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC) fastGetSetDCM.cpp

#Library Targets
libfastGetSetDCM.so: main.o fastGetSetDCM.o
	$(CXX) $(CXXFLAGS) -shared -o ../../../../Libs/libfastGetSetDCM.so main.o fastGetSetDCM.o $(NaoQi_LIB)

clean:
	rm -f *.o
	rm -f ../../../../Libs/libfastGetSetDCM.so 
    
all: fastGetSetDCM.o main.o libfastGetSetDCM.so
