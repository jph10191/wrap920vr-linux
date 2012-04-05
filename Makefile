CPP      = /usr/bin/g++
INCLUDEDIR = src/
CPPFLAGS  = -Wall -I$(INCLUDEDIR)
SRC = src/AttitudeSensor.cpp src/Head.cpp 

example: 
	$(CPP) $(CPPFLAGS) -o example $(SRC) 
