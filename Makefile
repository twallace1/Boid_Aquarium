EXE = aquarium
OBJS = aquarium.o parser.o boidsim.o ./lib/kdTree.o
CXXFLAGS = -Wall

all: $(EXE)

$(EXE) : $(OBJS)
	g++ $(CXXFLAGS) -o $(EXE) $(OBJS)
	cd lib; $(MAKE) $(MFLAGS)

clean :
	rm -f $(EXE) $(OBJS)
	rm -f *~ *.h.gch *#
