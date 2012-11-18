#include "Robot.hpp"

Robot::Robot(Controller& ctrl, Vision& vision)
: canReleaseBox(false) 
, stoppedForPicturesCounter(0)
, sawBoxCounter(0)
, boxDetected(0)
, hasBox(false)
, ctrl(&ctrl)
, vision(&vision)
, running(true)
{
        while (running)
        {
                vision->update();
                run(vision->detected_floor, vision->orig_small, vision->orig);
                vision->cleanupAfterUpdate();
        }
}

void Robot::moveBackConsideringFreeSpace(IplImage* detected_floor, int* odv)
{
        std::cout <<"Moving backwards...\n";
        int leftSpace = 0;
        for (size_t i = 0; i < detected_floor->width / 2; ++i)
        {
             leftSpace += odv[i];
        }
        int rightSpace = 0;
        for (size_t i = detected_floor->width / 2; i < detected_floor->width; ++i)
        {
             rightSpace += odv[i];
        }
        double leftSpaceAverage = leftSpace / (detected_floor->width / 2.0);
        double rightSpaceAverage = rightSpace / (detected_floor->width / 2.0);
        ctrl->moveBackward();
        usleep(400000);
        if (leftSpaceAverage < rightSpaceAverage)
        {
                ctrl->moveBackwardRight();
        }
        else
        {
                ctrl->moveBackwardLeft();
        }
        usleep(180000);
}

inline void Robot::stopAndRotate()
{
        std::cout << "stopping and rotating...\n";
        ctrl->stop(); 
        usleep(1000000);
        ctrl->rotateOnSpot();
        usleep(1000000);
}

inline void Robot::grabBox()
{
        std::cout << "-- Grabbing box\n";
        usleep(500000);
        ctrl->openServo();
        usleep(500000);
        ctrl->turn(0); 
        usleep(3000000);
        ctrl->closeServo();
        usleep(1000000);
        ctrl->stop();
        hasBox = true;
}



inline void Robot::dropBox(double angle)
{
        std::cout << "-- Dropping box\n";
        hasBox = false;
        ctrl->stop();
        usleep(5000000);
        std::cout << "base angle: " << angle << '\n';
        ctrl->turn(angle);
        usleep(100000);
        ctrl->turn(0); 
        usleep(35000 * (CAMERA_HEIGHT - baseCenterY));
        ctrl->openServo();
        ctrl->stop();
        ctrl->moveBackward();
        usleep(1000000);
        ctrl->rotateOnSpot();
        usleep(500000);
        ctrl->stop();
        usleep(5000000);
}


void Robot::run(IplImage* detected_floor, IplImage* normalCapture, IplImage* hdCapture)
{
        // write the moveable vector
        for (size_t i = 0; i < detected_floor->width; ++i)
        {
                if (odv[i] <= freeSpaceThreshold)
                {
                        moveable[i] = 0;
                }
                else
                {
                        moveable[i] = 1;
                }
        }

        // box detection
        BoxDetectionResult boxDetectionResult;
        if (!hasBox)
        {
                boxDetectionResult = detectBoxes(normalCapture, hdCapture, boxVec);
        }
        
        // can we grab the box?
        
        if (movementEnabled && 
            !hasBox && 
            boxDetectionResult.detected && 
            !boxDetectionResult.too_far && 
            boxDetectionResult.centered && 
            correct_box)
        {
                ctrl->stop();
                usleep(500000);
                grabBox(ctrl);
                return;
        }
        
        if (movementEnabled && hasBox && canReleaseBox)
        {
                int x_distance = baseCenterX - (normalCapture->width / 2);
                int y_distance = normalCapture->width - baseCenterY;
                double tan = (double)y_distance / (double)x_distance;
		double angle = std::atan(tan);
		angle = angle * 180 / PI;
		dropBox(ctrl, angle);
        }
        
        std::pair<size_t, size_t> boxLine = longestLine(boxVec, detected_floor->width);
              
        
        // determine if should move towards box        
        bool moveTowardsBox = false;
        int minBoxDistance = detected_floor->height;
        if (!hasBox && boxDetectionResult.detected) // only move towards box if it is too far
        {
                if (boxLine.second - boxLine.first == 0) 
                {
                        moveTowardsBox = false;
                }
                else
                {
                        moveTowardsBox = true;
                        std::cout << "Will move towards box.\n";
                        for (size_t i = boxLine.first; i < boxLine.second; ++i)
                        {
                                minBoxDistance = min(minBoxDistance, boxVec[i]);
                        }

                }
        }    

        // calculate the line, the centre of which we should move towards
        std::pair<size_t, size_t> moveableLine = longestLine(moveable, detected_floor->width);
        size_t beginMax = moveableLine.first;
        size_t endMax = moveableLine.second;
        size_t distanceMax = moveableLine.second - moveableLine.first;

        // centre of the line
        double movement_angle = 0;
        int goal_dist = 0;
        int lowestY = 0;

        if (moveTowardsBox)
        {
                lowestY = minBoxDistance;
                goal_dist = (boxLine.first + boxLine.second) / 2;
                std::cout << "moving towards box\n";
        }
        else
        {
                // calculate the minimum distance between the ground and the obstacles spanned by the line
                lowestY = detected_floor->height;
                for (size_t i = beginMax; i <= endMax; ++i)
                {
                        if (odv[i] < lowestY)
                                lowestY = odv[i];
                }
                goal_dist = (beginMax + endMax) / 2;
        
                // signed distance between centre of the line and centre of the image
                
        }

        int bottom_dist = goal_dist - (detected_floor->width / 2);
        if (bottom_dist != 0) 
        {

		        double tan = (double)lowestY / (double)bottom_dist;
		        movement_angle = std::atan(tan);
		        movement_angle = movement_angle * 180 / PI;
		        
		        if (movement_angle > 0)
		        { 
				movement_angle = 90 - movement_angle;
			}
		        if (movement_angle < 0)
		        { 
		        	movement_angle = -90 - movement_angle;
        		}
        }
        
        int leftIR = ctrl->getIRLeftValue();
        int rightIR = ctrl->getIRRightValue();
        
        bool moveBack = false;
        if (!moveTowardsBox)
        {
                if (distanceMax < 10)
                {
                        moveBack = true;
                        std::cout << "Moving back: camera facing wall\n";
                }
        }
        
        //std::cout << "left IR " << leftIR << '/' << irThreshold << '\n';
        if (leftIR > irThreshold)
        {
                moveBack = true;
                
        }
        if (rightIR > irThreshold)
        {
                moveBack = true;
        }
        //std::cout << "right IR " << rightIR << '/' << irThreshold << '\n';
        
        if (moveBack)
        {
                if (movementEnabled) 
                        moveBackConsideringFreeSpace(detected_floor, odv, ctrl);
                return;
        }
        
        if(distanceMax < 20)
        {
                if (movement_angle < 0)
                {
                	movement_angle = -90;
                }
                else
                {       
                	movement_angle = 90;
        	}
        }

        //std::cout << "counter: " << stoppedForPicturesCounter << ", angle: " << movement_angle << '\n';
        if (stoppedForPicturesCounter <= 1 || moveTowardsBox || hasBox)
        {
        	if (!lastMoveWasTowardsBox || moveTowardsBox)
        	{
        	        if (movementEnabled) 
        	        {
                	        if (hasBox) 
                	        {
                	                ctrl->turn(movement_angle);
                	        }
                	        else
                	        {
                	                ctrl->turnAt(movement_angle);
                	        }
                	}        
        	}
        	        
        	if (moveTowardsBox)
        	{
        	        usleep(100000);
        	        lastMoveWasTowardsBox = true;
        	}       
        	else
        	{
        	        lastMoveWasTowardsBox = false;
        	}

        }        
}

static std::pair<size_t, size_t> Robot::longestLine(int* vec, size_t size)
{
        size_t beginMax = 0;
        size_t distanceMax = 0;
        size_t endMax = 0;
                
        size_t begin = 0;
        size_t distance = 0;
        size_t end = 0;
        
        bool current = false;
        
        for (size_t i = 0; i < size; ++i)
        {
                if (current && i == size - 1 && vec[i])
                {
                        end = i;
                        if (distance > distanceMax)
                        {
                                beginMax = begin;
                                endMax = end;
                                distanceMax = distance;
                        }
                }
                if (vec[i])
                {
                        if (current)
                        {
                                distance++;
                                end = i;
                        }
                        else
                        {
                                distance = 1;
                                begin = i;
                                end = begin;
                                current = true;
                        }
                }
                else
                {
                        if (current)
                        {
                                current = false;
                                if (distance > distanceMax)
                                {
                                        beginMax = begin;
                                        endMax = end;
                                        distanceMax = distance;
                                }
                        }
                        
                }
        }
        std::pair<size_t, size_t> ret;
        ret.first = beginMax;
        ret.second = endMax;
        return ret;
}

