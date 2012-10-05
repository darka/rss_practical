#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/gpu/gpu.hpp"
#include <cv_bridge/CvBridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    if(!msg->data.empty()){

        /* Convert ROS image data to OpenCV image data */
        sensor_msgs::CvBridge bridge;
        cv::Mat image(bridge.imgMsgToCv(msg, "bgr8"));
        cv::gpu::GpuMat gpuImage;
        gpuImage.upload(image);
        /* Initialize OpenCV HOG classifier */
        //cv::gpu::HOGDescriptor hog;
        cv::HOGDescriptor hog;
        hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        std::vector<cv::Rect> BoundingBox, BoundingBoxFiltered;
        //hog.detectMultiScale(gpuImage, BoundingBox, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 2);
        hog.detectMultiScale(image, BoundingBox, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 2);


        /* Map bounding boxes to image */
        size_t i, j;
        for (i=0; i<BoundingBox.size(); i++)
         {
             cv::Rect r = BoundingBox[i];
             for (j=0; j<BoundingBox.size(); j++)
                 if (j!=i && (r & BoundingBox[j])==r)
                     break;
             if (j==BoundingBox.size())
                 BoundingBoxFiltered.push_back(r);
         }
        for (i=0; i<BoundingBoxFiltered.size(); i++)
                {
                cv::Rect r = BoundingBoxFiltered[i];
                    r.x += cvRound(r.width*0.1);
                r.width = cvRound(r.width*0.8);
                r.y += cvRound(r.height*0.06);
                r.height = cvRound(r.height*0.9);
                cv::rectangle(image, r.tl(), r.br(), cv::Scalar(0,255,0), 2);
            }
        cv::imshow("view", image);

    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cvNamedWindow("view");
  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("out", 1, imageCallback);
  ros::spin();
  cvDestroyWindow("view");
}
