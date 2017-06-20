OBJS = kdTree.o slVector.o
CXXFLAGS = -Wall

all: $(OBJS)

kdTree.o : kdTree.cpp
	g++ $(CXXFLAGS) -c kdTree.cpp

slVector.o : slVector.cpp
	g++ $(CXXFLAGS) -c slVector.cpp

clean :
	rm -f *.o
	rm -f *~ *.h.gch *#
