#ifndef CAMERALISTENER_HPP
#define CAMERALISTENER_HPP

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvwimage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>


namespace HRICS
{
    class CameraListener
    {
        public:
            CameraListener();
            CameraListener(const int id);
            image_transport::Subscriber _sub;
            image_transport::Publisher _pub;
            int _file;
            void imageConverter(const sensor_msgs::ImageConstPtr& msg);
            void setId(int id) {_id = id;}
            cv_bridge::CvImagePtr _current_img;
            void takeSnapshot(timeval time);
            void pubImage(timeval time);


            bool _is_recording;


        private:
            int _id;


    };
}

#endif // CAMERALISTENER_HPP
