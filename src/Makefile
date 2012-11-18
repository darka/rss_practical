CC=g++
CFLAGS=-O3 -g -pg
LDFLAGS=-lphidget21 -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_features2d
SOURCES=Controller.cpp VideoStream.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=VideoStream

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) -o $@ $(OBJECTS) $(LDFLAGS) 

.cpp.o:
	$(CC) -c $< -o $@ $(CFLAGS) 

clean:
	rm -rf $(EXECUTABLE) $(OBJECTS)
	

