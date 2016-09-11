CPP_FLAGS=-Wall -g 

all: TransformTestAlex

TransformTestAlex: TransformTestAlex.o Transform.o
	g++ -o TransformTestAlex TransformTestAlex.o Transform.o $(CPP_FLAGS)

Transform.o: Transform.cpp Transform.h
	g++ -c Transform.cpp  $(CPP_FLAGS)

TransformTestAlex.o: TransformTestAlex.cpp
	g++ -c TransformTestAlex.cpp $(CPP_FLAGS)
	
clean:
	rm -f *o TransformTestAlex
