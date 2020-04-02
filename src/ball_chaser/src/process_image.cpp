#include "ros/ros.h"
#include <tuple>

#include "sensor_msgs/Image.h"

#include "ball_chaser/DriveToTarget.h"

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
        std::pair<float, float> FindRegion(int rows, int height, int pixel_index);
};

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
       std::pair<float, float> result = ImageProcessor::FindRegion(img->step, img->height, pixel_index); 
       ImageProcessor::DriveBot(result.first, result.second);
    } else {  // if no ball found stop the robot.
        ImageProcessor::DriveBot(0, 0);
    }
    

}

std::pair<float, float> ImageProcessor::FindRegion(int column, int row, int pixel_index) {
    float row_pos = float((pixel_index - (pixel_index/row)*row))/float(column);
    // if the ball is in the left portion of the robot move left
    if (row_pos <= 0.36) {
        return std::make_pair(0.1,0.1); 
    //  if the ball is in the center go straight
    } else if ((row_pos>0.36)&&(row_pos<0.65)) {
        return std::make_pair(0.1, 0.0);
    // if the ball is in the right region move and rotate right
    } else if (row_pos >= 0.65) {
        return std::make_pair(0.1, -0.1); 
    }
}

void ImageProcessor::DriveBot(float linear_x, float angular_z) {
    ROS_INFO_STREAM("Robot chassing the ball");
    ball_chaser::DriveToTarget srv;
    // set the request values to linear and angular argument values
    srv.request.linear_x = linear_x;
    srv.request.angular_z = angular_z;

    // call the service /drive_robot/command
    if(!this->client_.call(srv)) {
        ROS_ERROR("Failed to call the service");
    }
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "image_processing");
    
    // initialize the image processor node by creating an object
    ImageProcessor image_processor;
    // handle ros communication events
    ros::spin();

    return 0;
}