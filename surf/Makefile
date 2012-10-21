Surf.o: main.cpp
	g++ -c main.cpp -o Surf.o -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lphidget21 -lopencv_calib3d -lopencv_features2d

Surf: Surf.o 
	g++ -o Surf Surf.o -lphidget21  -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_features2d
	
sift.o: sift.cpp
	g++ -c sift.cpp -o sift.o -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lphidget21 -lopencv_calib3d -lopencv_features2d

sift: sift.o 
	g++ -o sift sift.o -lphidget21  -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_features2d
	
	
edge_detection.o: edge_detection.cpp
	g++ -c edge_detection.cpp -o edge_detection.o -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lphidget21 -lopencv_calib3d -lopencv_features2d -O3

edge_detection: edge_detection.o 
	g++ -o edge_detection edge_detection.o -lphidget21  -lopencv_core -lopencv_video -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_features2d -O3
	
clean:
	rm -rf Surf sift edge_detection
