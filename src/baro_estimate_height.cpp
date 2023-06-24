#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/FluidPressure.h>

#include <deque>

class BarometricToHeightNode
{
public:
  // Constructor
  BarometricToHeightNode() : nh_(), nh_param_("~"), height_(0.0),  num_samples_(0), num_initial_samples_(50) // Initialize height to 0 and set window size
  {
    barometric_sub_ = nh_.subscribe("/mavros/imu/static_pressure", 1, &BarometricToHeightNode::barometricCallback, this);
    height_pub_ = nh_.advertise<std_msgs::Float32>("height", 10);


    if(!nh_param_.getParam("/baro_estimate_height_node/window_size",window_size_))
    {
      std::cout<<"[Warning] Please set windows_size parameter, default : 25"<<std::endl;
      window_size_ = 25;
    }
  }

  //barometric callback
  void barometricCallback(const sensor_msgs::FluidPressure::ConstPtr& msg)
  {
    // Check if height initialization is complete
    if (!height_initialized_)
    {
      initializeHeight(msg->fluid_pressure);
    }
    
    // Apply moving average filter to barometric pressure data
    barometric_buffer_.push_back(msg->fluid_pressure);

    if (barometric_buffer_.size() > window_size_)
      barometric_buffer_.pop_front();

    float filtered_pressure = calculateMovingAverage();

  
    height_ = (filtered_pressure - initial_average_pressure_) * conversion_factor_;

    // Publish the calculated height
    // TODO :: remove abs(height).
    if(height_initialized_ && abs(height_)<100)
    {
      std_msgs::Float32 height_msg;
      height_msg.data = height_;
      height_pub_.publish(height_msg);
    }
  }

  void initializeHeight(float initial_pressure)
  {
    if(num_samples_ == 0)
    {
      ROS_INFO("Start height initialization, number of collecting sample : %d", num_initial_samples_);
    }
    initial_pressure_sum_ += initial_pressure;
    num_samples_++;

    if (num_samples_ >= num_initial_samples_)
    {
      initial_average_pressure_ = initial_pressure_sum_ / num_initial_samples_;
  
      height_initialized_ = true;

      ROS_INFO("Height initialization complete. Average pressure: %.2f", initial_average_pressure_);
    }
  }


  float calculateMovingAverage()
  {
    float sum = 0.0;
    for (const auto& pressure : barometric_buffer_)
    {
      sum += pressure;
    }

    return sum / barometric_buffer_.size();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_param_;
  ros::Subscriber barometric_sub_;
  ros::Publisher height_pub_;

  std::deque<float> barometric_buffer_; // Buffer to store barometric pressure data for moving average
  float height_;                        // Current height estimate
  int window_size_;                     // Size of the moving average window
  float conversion_factor_ = -0.1;       // Conversion factor from pressure to height


  bool height_initialized_;        // Flag to indicate if height is initialized
  int num_initial_samples_;        // Number of initial samples for averaging
  int num_samples_;                // Counter for the number of samples
  float initial_pressure_sum_;     // Accumulator for initial pressure sum
  float initial_average_pressure_; 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "barometric_to_height_node");

  BarometricToHeightNode node;

  ros::spin();

  return 0;
}
