#include "cameraListener.hpp"


using namespace std;
using namespace HRICS;

CameraListener::CameraListener()
{
    cout << "Enter constructer for camera" << endl;
    setId(0);
    int argc = 0;
    char** argv;
    ros::init(argc, argv, "image_saver");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    _is_recording = false;
    _file = 0;


    cout << "start suscriber" << endl;
    _sub = it.subscribe("camera/rgb/image_raw", 1, &CameraListener::imageConverter, this);

    cout << "start publisher" << endl;
    _pub = it.advertise("orkinect/image", 1);

}

CameraListener::CameraListener(const int id)
{
    cout << "Enter constructer for camera" << endl;
    setId(id);
    _is_recording = false;

    int argc = 0;
    char** argv;
    ros::init(argc, argv, "image_saver");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    _file = 0;


    cout << "start suscriber" << endl;
    std::stringstream s;
    s << "camera" << _id << "/rgb/image_raw";
    _sub = it.subscribe(s.str().c_str(), 1, &CameraListener::imageConverter, this);

    cout << "start publisher" << endl;
    s.str( "" );
    s.clear();

    s << "orkinect/kinect" << _id;
    _pub = it.advertise(s.str().c_str(), 1);

}

void CameraListener::imageConverter(const sensor_msgs::ImageConstPtr& msg)
{
    _current_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

}

void CameraListener::takeSnapshot(timeval time)
{
    try
    {
        std::stringstream s;
        s << "/home/rafihayne/workspace/statFiles/snapshots/" << _id << "_" << time.tv_sec << "_" << time.tv_usec << ".png";
//        s << "/home/rafihayne/workspace/statFiles/snapshots/" << _id << "_" << _file++ << ".png";
        cv::imwrite(s.str().c_str(), _current_img->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void CameraListener::pubImage(timeval time)
{
    std::stringstream file;
    file << "/home/rafihayne/workspace/statFiles/snapshots/" << _id << "_" << time.tv_sec << "_" << time.tv_usec << ".png";
//    file << "/home/rafihayne/workspace/statFiles/snapshots/" << _id << "_" << frame << ".png";

    cv::WImageBuffer3_b image( cvLoadImage(file.str().c_str(), CV_LOAD_IMAGE_COLOR) );
    cv::Mat imageMat(image.Ipl());

    cv_bridge::CvImage out_msg;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image    = imageMat;
    out_msg.header.stamp = ros::Time::now();

    _pub.publish(out_msg.toImageMsg());
}

