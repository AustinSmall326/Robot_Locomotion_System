#Variables
CXXFLAGS=-Wall -g -fPIC
CXX = g++
NaoQi_INC = /home/nao/naoqi-sdk-2.1.3.3-linux32/include
NaoQi_LIB = -L/home/nao/naoqi-sdk-2.1.3.3-linux32/lib -lalcommon -lalerror -lalproxies -lalmemoryfastaccess -lalvalue

#Object Targets
main.o: main.cpp shm.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC)  main.cpp

shm.o: shm.cpp shm.h $(NaoQi_INC)/alcommon/alproxy.h $(NaoQi_INC)/alcommon/albroker.h $(NaoQi_INC)/alcommon/almodule.h \
                     $(NaoQi_INC)/alerror/alerror.h $(NaoQi_INC)/alproxies/almemoryproxy.h \
                     $(NaoQi_INC)/almemoryfastaccess/almemoryfastaccess.h $(NaoQi_INC)/alvalue/alvalue.h
	$(CXX) $(CXXFLAGS) -c -I$(NaoQi_INC) shm.cpp

#Library Targets
libshm.so: main.o shm.o
	$(CXX) $(CXXFLAGS) -shared -o ../../Libs/libshm.so main.o shm.o $(NaoQi_LIB)

clean:
	rm -f *.o
	rm -f ../../Libs/libshm.so 
    
all: shm.o main.o libshm.so
