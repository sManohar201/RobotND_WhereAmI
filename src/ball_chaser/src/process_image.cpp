#include "ros/ros.h"
#include <tuple>
#include "ball_cahser/DriveToTarget.h"

class ImageProcessor {
    private:
        // define a node handle for this node        
        ros::NodeHandle n_;
        // define service client object
        ros::ServiceClient client_;
        // define subscriber object
        ros::Subscriber sub_;
        bool ball_displaced_;
        
    public:
        ImageProcessor();
        ~ImageProcessor() {};
        // subscriber callback function
        void ProcessImageCallback(const sensor_msgs::Image::ConstPtr &img);
        // service client caller function.
        void DriveBot(float linear_x, float angular_z);
        // identify where the ball is in the image
        std::pair FindRegion(const sensor_msgs::Image::ConstPtr &img, int pixel_index);
}

ImageProcessor::ImageProcessor() {
    // create client object using the node handle
    client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/drive_robot/command");
    // create subscriber object which borrows image data
    sub_ = n_.subscribe("/camera/rgb/image_raw", 10, &ImageProcessor::ProcessImageCallback, this);
    // initialize ball_displaced to false
    ball_displaced_ = false;
}

void ImageProcessor::ProcessImageCallback(const sensor_msgs::Image::ConstPtr &img) {
    int white_pixel = 255;
    int pixel_index;
    // loop through each image and find if the robot sees a ball
    for (int i=0; i<img->height*img->step; i++) {
        if (img->data[i] == white_pixel) {
            ball_displaced_ = true;
            pixel_index = i;
            break;
        }
    }

    if (ball_displaced_) {
       std::pair<float, float> result = ImageProcessor::FindRegion(img, pixel_index); 
    }

}

std::pair ImageProcessor::FindRegion(const sensor_msgs::Image::ConstPtr &img, int pixel_index) {
    
}

void ImageProcessor::DriveBot(float linear_x, float angular_z) {

}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "image_processing");
    
    // initialize the image processor node by creating an object
    ImageProcessor image_processor;
    // handle ros communication events
    ros::spin();

    return 0;
}