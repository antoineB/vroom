
INCDIRS=-I/usr/local/include/ -I/usr/include/ -I/usr/local/include/OGRE/
LIBSDIRS=-L/usr/local/lib/ -L.
LIBS=-lOgreMain -lOIS -lCEGUIOgreRenderer -lode -ltinyxml
CXXFLAGS=-Wall -Wextra -pedent
CXXFLAGS=
CXX=g++

main: main.cpp demo.o OgreFramework.o car.o wheel.o mytools.o ground.o world.o space.o obstacle.o geom.o movableobstacle.o type.o utils.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(INCDIRS) $(LIBSDIRS) $(LIBS)

demo.o: demo.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

OgreFramework.o: OgreFramework.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

car.o: car.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

wheel.o: wheel.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

mytools.o: mytools.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

ground.o: ground.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

world.o: world.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

space.o: space.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

obstacle.o: obstacle.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

geom.o: geom.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

movableobstacle.o: movableobstacle.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

type.o: type.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

utils.o: utils.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

.PHONY: check-syntax
check-syntax:
	 $(CXX) -Wall -Wextra -pedantic -fsyntax-only $(CHK_SOURCES)

clean:
	rm *~ *.o main *.log