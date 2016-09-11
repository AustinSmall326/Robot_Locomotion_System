#Variables
CXXFLAGS=-Wall -g 
CXX = g++

#Executable

#Object Targets
TransformTests.o: TransformTests.cpp TransformTests.h Transform.h
	$(CXX) $(CXXFLAGS) -c TransformTests.cpp

Transform.o: Transform.cpp Transform.h
	$(CXX) $(CXXFLAGS) -c Transform.cpp

TrajectoryTests.o: TrajectoryTests.cpp TrajectoryTests.h Trajectory.h Transform.h
	$(CXX) $(CXXFLAGS) -c TrajectoryTests.cpp

Trajectory.o: Trajectory.cpp Trajectory.h Transform.h
	$(CXX) $(CXXFLAGS) -c Trajectory.cpp

PointTests.o: PointTests.cpp PointTests.h Point.h Trajectory.h Transform.h
	$(CXX) $(CXXFLAGS) -c PointTests.cpp

Point.o: Point.cpp Point.h Trajectory.h Transform.h COMContainer.h
	$(CXX) $(CXXFLAGS) -c Point.cpp

COMContainerTests.o: COMContainerTests.cpp COMContainerTests.h COMContainer.h
	$(CXX) $(CXXFLAGS) -c COMContainerTests.cpp

COMContainer.o: COMContainer.cpp COMContainer.h
	$(CXX) $(CXXFLAGS) -c COMContainer.cpp

clean:
	rm -f *.o
    
all: Transform.o TransformTests.o Trajectory.o TrajectoryTests.o Point.o PointTests.o COMContainer.o COMContainerTests.o 
