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
        int pixel_index_column_;

    public:
        ImageProcessor();
        ~ImageProcessor() {};
        // subscriber callback function
        void ProcessImageCallback(const sensor_msgs::Image::ConstPtr &img);
        // service client caller function.
        void DriveBot(float linear_x, float angular_z);
        // identify where the ball is in the image
        void FindRegion(int);
};

ImageProcessor::ImageProcessor() {
    // create client object using the node handle
    client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/drive_robot/command");
    // create subscriber object which borrows image data
    sub_ = n_.subscribe("/camera/rgb/image_raw", 10, &ImageProcessor::ProcessImageCallback, this);
    // initialize ball_displaced to false
}

void ImageProcessor::ProcessImageCallback(const sensor_msgs::Image::ConstPtr &img) {
    int white_pixel = 255;
    // loop through each image and find if the robot sees a ball
     int row = 0;
    int step = 0;
    int i = 0;

    bool ball_displaced = false;
    //ROS_INFO("height: %d, width: %d, step: %d", img.height, img.width, img.step);
    //ROS_INFO("HEIGHT: %f, STEP: %f", img.height, img.step);
    for (row = 0; row < img->height && ball_displaced == false; row++)
    {
        for (step = 0; step < (img->step-2) && ball_displaced == false; )
        {   
            i = (row*img->step)+step;
            //ROS_INFO("row: %d, step: %d, i: %d", row, step, i);
            if ((img->data[i] == white_pixel)&&(img->data[i+1] == white_pixel)&&(img->data[i+2] == white_pixel))
            {   
                ball_displaced = true;
                //ROS_INFO("row: %d, step: %d, i: %d", row, step, i);
                pixel_index_column_ = step;
            }
            step = step+3;
		}
    }
   
       if (ball_displaced) {
        ImageProcessor::FindRegion(img->width); 
    } else {  // if no ball found stop the robot.
        ImageProcessor::DriveBot(0, 0);
    }
}

void ImageProcessor::FindRegion(int width) {

    int row_pos = pixel_index_column_/3;
    int whole_Img = width/3; 
    ROS_INFO("Who : %d , pos : %d", whole_Img, row_pos);
    float linear, angular;
    // if the ball is in the left portion of the robot move left
    if (row_pos <= whole_Img) {
        linear = 0.1;
        angular = 0.05;
        ROS_INFO_STREAM("Ball in left portion");
    //  if the ball is in the center go straight
    } else if ((row_pos>whole_Img)&&(row_pos<2*whole_Img)) {
        linear = 0.1;
        angular = 0; 
        ROS_INFO_STREAM("Ball in the middle portion");
    // if the ball is in the right region move and rotate right
    } else if (row_pos >= 2*whole_Img) {
        linear = 0.1; 
        angular = -0.05;
        ROS_INFO_STREAM("Ball in the right portion");
    }
    ImageProcessor::DriveBot(linear, angular);
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