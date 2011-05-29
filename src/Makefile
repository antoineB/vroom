
INCDIRS=-I/usr/local/include/ -I/usr/include/ -I/usr/local/include/OGRE/
LIBSDIRS=-L/usr/local/lib/ -L.
LIBS=-lOgreMain -lOIS -lCEGUIOgreRenderer -lode -ltinyxml
CXXFLAGS=-Wall -Wextra -pedent
CXXFLAGS=
CHK_SOURCES=%.cpp %.hpp

main: main.cpp demo.o OgreFramework.o car.o wheel.o mytools.o ground.o world.o space.o obstacle.o geom.o movableobstacle.o type.o utils.o
	g++ $(CXXFLAGS) -o $@ $^ $(INCDIRS) $(LIBSDIRS) $(LIBS)

demo.o: demo.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

OgreFramework.o: OgreFramework.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

car.o: car.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

wheel.o: wheel.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

mytools.o: mytools.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

ground.o: ground.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

world.o: world.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

space.o: space.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

obstacle.o: obstacle.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

geom.o: geom.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

movableobstacle.o: movableobstacle.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

type.o: type.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)

utils.o: utils.cpp
	g++ $(CXXFLAGS) -o $@ -c $^ $(INCDIRS)


check-syntax:
	g++ -Wall -Wextra -pedent -fsyntax-only $(CHK_SOURCES) $(INCDIRS)

clean:
	rm *~ *.o main *.log