#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "opencv_apps/FaceArrayStamped.h"
#include "opencv_apps/Rect.h"
#include <cmath>

const std::string face_detector_group = "/face_det";
const double cam_width = 800; // Pixels
ros::Publisher locationPublisher;
ros::Publisher validPublisher;

static double inline deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

static double getErrorCam(double position, double bound1, double bound2) {
    double center = (bound1 + bound2)/2;
    double width = (bound2 - bound1)/2;
    double error = (position - center)/width;
    return error;
}

void faceDetectedCallback(opencv_apps::FaceArrayStamped face_array) {
    std_msgs::Bool valid;
    if (face_array.faces.size() == 0) {
        valid.data = false;
        validPublisher.publish(valid);
        return;
    }
    // If we have a face, then its valid
    valid.data = true;
    validPublisher.publish(valid);

    // Find the angular error from the center
    opencv_apps::Rect bounding_rect = face_array.faces[0].face;
    std_msgs::Float64 goal;
    goal.data = -deg2rad(78.0/2) * getErrorCam(bounding_rect.x, 0, cam_width);
    locationPublisher.publish(goal);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "face_tracker_node");
    ros::start();

    ros::NodeHandle nodeHandler;

    locationPublisher =
        nodeHandler.advertise<std_msgs::Float64>("/goal", 1);

    validPublisher =
        nodeHandler.advertise<std_msgs::Bool>("/valid", 1);

    ros::Subscriber faceSubscriber =
        nodeHandler.subscribe(face_detector_group + "/faces", 10,
                              &faceDetectedCallback);

    ros::spin();

    ros::shutdown();

    return 0;
}
