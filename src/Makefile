Eight: Eight.c
	g++ Eight.c -o Eight -lphidget21

CameraCapture: CameraCapture.cpp
	g++ CameraCapture.cpp -o CameraCapture -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc
	
EdgeDetection: EdgeDetection.cpp
	g++ EdgeDetection.cpp -o EdgeDetection -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc

sift: sift.cpp
	g++ sift.cpp -o sift -lphidget21 -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lopencv_features2d
	
Controller.o: Controller.cpp Controller.hpp
	g++ -c Controller.cpp -o Controller.o -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lphidget21
	
VideoStream.o: VideoStream.cpp Controller.o
	g++ -c VideoStream.cpp -o VideoStream.o -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lphidget21  -lopencv_calib3d -lopencv_features2d

VideoStream: VideoStream.o Controller.o
	g++ -o VideoStream VideoStream.o Controller.o  -lphidget21  -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_features2d
		
Main.o: Main.cpp Controller.o
	g++ -c Main.cpp -o Main.o Controller.o -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lphidget21

all: Main.o Controller.o
	g++ -o Main Main.o Controller.o  -lphidget21  -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc 
	
clean:
	rm -rf EdgeDetection CameraCapture VideoStream Eight Main Controller
