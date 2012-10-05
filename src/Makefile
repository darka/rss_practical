Eight: Eight.c
	g++ Eight.c -o Eight -lphidget21

VideoStream: VideoStream.cpp
	g++ VideoStream.cpp -o VideoStream -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc
	
CameraCapture: CameraCapture.cpp
	g++ CameraCapture.cpp -o CameraCapture -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc
	
EdgeDetection: EdgeDetection.cpp
	g++ EdgeDetection.cpp -o EdgeDetection -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc
	
Controller.o: Controller.cpp Controller.hpp
	g++ -c Controller.cpp -o Controller.o -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lphidget21

Main.o: Main.cpp Controller.o
	g++ -c Main.cpp -o Main.o Controller.o -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lphidget21

all: Main.o Controller.o
	g++ -o Main Main.o Controller.o  -lphidget21
	
clean:
	rm -rf EdgeDetection CameraCapture VideoStream Eight Main Controller
