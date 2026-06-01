#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <map>
#include <algorithm>

class CompressedDepthImageViewerNode : public rclcpp::Node
{
public:
  CompressedDepthImageViewerNode() : Node("compressed_depth_image_viewer_node")
  {
    // Declare parameters
    this->declare_parameter<int>("window_width", 640);
    this->declare_parameter<int>("window_height", 480);
    this->declare_parameter<int>("window_type", cv::WINDOW_NORMAL);
    this->declare_parameter<std::string>("window_name", "Depth Image Viewer");
    this->declare_parameter<bool>("display_info", false);
    this->declare_parameter<bool>("record_frames", false);
    this->declare_parameter<std::string>("record_path", "/tmp/depth_frames/");
    this->declare_parameter<bool>("apply_colormap", true);
    this->declare_parameter<int>("colormap_type", cv::COLORMAP_JET);
    this->declare_parameter<bool>("normalize_depth", true);

    // Get parameter values
    window_width_ = this->get_parameter("window_width").as_int();
    window_height_ = this->get_parameter("window_height").as_int();
    window_type_ = this->get_parameter("window_type").as_int();
    window_name_ = this->get_parameter("window_name").as_string();
    display_info_ = this->get_parameter("display_info").as_bool();
    record_frames_ = this->get_parameter("record_frames").as_bool();
    record_path_ = this->get_parameter("record_path").as_string();
    apply_colormap_ = this->get_parameter("apply_colormap").as_bool();
    colormap_type_ = this->get_parameter("colormap_type").as_int();
    normalize_depth_ = this->get_parameter("normalize_depth").as_bool();

    RCLCPP_INFO(this->get_logger(), 
      "Depth Image Viewer initialized - Window: %s, Size: %dx%d, Type: %d",
      window_name_.c_str(), window_width_, window_height_, window_type_);
    RCLCPP_INFO(this->get_logger(), 
      "Display Info: %s, Record: %s, Colormap: %s, Normalize: %s",
      display_info_ ? "true" : "false", 
      record_frames_ ? "true" : "false",
      apply_colormap_ ? "true" : "false",
      normalize_depth_ ? "true" : "false");

    // Create subscription to compressed depth image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/depth/image_raw/compressedDepth",
      rclcpp::SensorDataQoS(),
      std::bind(&CompressedDepthImageViewerNode::image_callback, this, std::placeholders::_1));

    // Create window with configured parameters
    cv::namedWindow(window_name_, window_type_);
    cv::resizeWindow(window_name_, window_width_, window_height_);
    
    // Set mouse callback for tracking window events
    cv::setMouseCallback(window_name_, mouse_callback, this);

    // Create recording directory if needed
    if (record_frames_)
    {
      system(("mkdir -p " + record_path_).c_str());
    }
  }

  ~CompressedDepthImageViewerNode()
  {
    cv::destroyAllWindows();
  }

private:
  int window_width_;
  int window_height_;
  int window_type_;
  std::string window_name_;
  bool display_info_;
  bool record_frames_;
  std::string record_path_;
  bool apply_colormap_;
  int colormap_type_;
  bool normalize_depth_;
  cv::Mat current_image_;
  cv::Mat current_depth_raw_;
  int frame_count_ = 0;

  // Decode ROS2 compressedDepth format using cv_bridge
  cv::Mat decodeDepthImage(const sensor_msgs::msg::CompressedImage::SharedPtr& msg)
  {
    try
    {
      // Use cv_bridge to decompress the depth image
      // The compressedDepth format is handled by cv_bridge
      cv_bridge::CvImagePtr cv_ptr;
      
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->format);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        
        // Fallback: try to decompress as generic compressed image
        try
        {
          cv_ptr = cv_bridge::toCvCopy(msg, "");
        }
        catch (cv_bridge::Exception& e2)
        {
          RCLCPP_ERROR(this->get_logger(), "cv_bridge fallback exception: %s", e2.what());
          return cv::Mat();
        }
      }

      if (!cv_ptr || cv_ptr->image.empty())
      {
        RCLCPP_WARN(this->get_logger(), 
          "Failed to decode compressed depth image. Format: %s, Data size: %zu bytes",
          msg->format.c_str(), msg->data.size());
        return cv::Mat();
      }

      return cv_ptr->image.clone();
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error decoding depth image: %s", e.what());
      return cv::Mat();
    }
  }

  cv::Mat processDepthImage(const cv::Mat& depth_raw)
  {
    cv::Mat processed = depth_raw.clone();

    // Ensure 8-bit for display
    if (processed.type() != CV_8U)
    {
      if (normalize_depth_)
      {
        double min_val, max_val;
        cv::minMaxLoc(processed, &min_val, &max_val);
        
        if (max_val > min_val)
        {
          processed.convertTo(processed, CV_8U, 255.0 / (max_val - min_val), 
                            -min_val * 255.0 / (max_val - min_val));
        }
        else
        {
          processed.convertTo(processed, CV_8U);
        }
      }
      else
      {
        // Convert to 8-bit without normalization (for 16-bit depth)
        processed.convertTo(processed, CV_8U, 255.0 / 65535.0);
      }
    }

    // Apply colormap if enabled
    if (apply_colormap_)
    {
      cv::Mat colored;
      cv::applyColorMap(processed, colored, colormap_type_);
      return colored;
    }

    return processed;
  }

  void drawImageInfo(cv::Mat& image, const sensor_msgs::msg::CompressedImage::SharedPtr& msg)
  {
    if (!display_info_)
      return;

    const int font = cv::FONT_HERSHEY_SIMPLEX;
    const double font_scale = 0.5;
    const int thickness = 1;
    const cv::Scalar text_color(0, 255, 0);
    const cv::Scalar bg_color(0, 0, 0);
    const int line_height = 20;
    const int margin = 5;

    // Prepare info text
    std::vector<std::string> info_lines;
    info_lines.push_back("Frame: " + std::to_string(frame_count_));
    info_lines.push_back("Size: " + std::to_string(current_depth_raw_.cols) + "x" + 
                         std::to_string(current_depth_raw_.rows));
    info_lines.push_back("Format: " + msg->format);
    info_lines.push_back("Compressed Size: " + std::to_string(msg->data.size()) + " bytes");
    
    std::string depth_type;
    int cv_type = current_depth_raw_.type();
    if (cv_type == CV_16U) depth_type = "16UC1 (uint16)";
    else if (cv_type == CV_16S) depth_type = "16SC1 (int16)";
    else if (cv_type == CV_32F) depth_type = "32FC1 (float32)";
    else if (cv_type == CV_8U) depth_type = "8UC1 (uint8)";
    else depth_type = "Unknown";
    info_lines.push_back("Depth Type: " + depth_type);

    // Calculate total text size
    int y_offset = margin + 15;
    for (const auto& line : info_lines)
    {
      cv::Size text_size = cv::getTextSize(line, font, font_scale, thickness, nullptr);
      
      // Draw background rectangle
      cv::rectangle(image, 
                   cv::Point(margin - 2, y_offset - text_size.height - 2),
                   cv::Point(margin + text_size.width + 2, y_offset + 2),
                   bg_color, -1);
      
      // Draw text
      cv::putText(image, line, cv::Point(margin, y_offset), 
                 font, font_scale, text_color, thickness);
      
      y_offset += line_height;
    }
  }

  void recordFrame(const cv::Mat& image)
  {
    if (!record_frames_)
      return;

    try
    {
      std::string filename = record_path_ + "depth_frame_" + 
                           std::to_string(frame_count_) + ".png";
      bool success = cv::imwrite(filename, image);
      
      if (!success)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to save depth frame to %s", filename.c_str());
      }
      else if (frame_count_ % 30 == 0)
      {
        RCLCPP_DEBUG(this->get_logger(), "Saved depth frame to %s", filename.c_str());
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error recording depth frame: %s", e.what());
    }
  }

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    try
    {
      // Decode compressed depth image
      cv::Mat depth_raw = decodeDepthImage(msg);

      if (depth_raw.empty())
      {
        return;
      }

      // Store raw depth image
      current_depth_raw_ = depth_raw.clone();
      frame_count_++;

      // Process depth image (normalize and apply colormap)
      cv::Mat display_image = processDepthImage(depth_raw);

      // Store for mouse callback
      current_image_ = display_image.clone();

      // Draw info on image if enabled
      drawImageInfo(display_image, msg);

      // Record frame if enabled
      recordFrame(display_image);

      // Display image
      cv::imshow(window_name_, display_image);

      // Process key press (ESC to exit, 'S' to save, 'R' to toggle recording, 'C' to toggle colormap)
      int key = cv::waitKey(1);
      if (key == 27)  // ESC key
      {
        RCLCPP_INFO(this->get_logger(), "ESC key pressed, shutting down...");
        rclcpp::shutdown();
      }
      else if (key == 's' || key == 'S')  // Save current frame
      {
        std::string filename = "/tmp/depth_frame_" + 
                             std::to_string(frame_count_) + ".png";
        cv::imwrite(filename, current_image_);
        RCLCPP_INFO(this->get_logger(), "Saved depth frame to %s", filename.c_str());
      }
      else if (key == 'r' || key == 'R')  // Toggle recording
      {
        record_frames_ = !record_frames_;
        RCLCPP_INFO(this->get_logger(), "Recording %s", 
                   record_frames_ ? "enabled" : "disabled");
        if (record_frames_)
        {
          system(("mkdir -p " + record_path_).c_str());
        }
      }
      else if (key == 'c' || key == 'C')  // Toggle colormap
      {
        apply_colormap_ = !apply_colormap_;
        RCLCPP_INFO(this->get_logger(), "Colormap %s", 
                   apply_colormap_ ? "enabled" : "disabled");
      }

      // Log frame info occasionally
      if (frame_count_ % 30 == 0)
      {
        RCLCPP_DEBUG(this->get_logger(), 
          "Depth Frame %d: %dx%d, format: %s, data size: %zu bytes",
          frame_count_, depth_raw.cols, depth_raw.rows, msg->format.c_str(), msg->data.size());
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error processing depth image: %s", e.what());
    }
  }

  static void mouse_callback(int event, int x, int y, int flags, void* userdata)
  {
    CompressedDepthImageViewerNode* node = static_cast<CompressedDepthImageViewerNode*>(userdata);
    
    if (event == cv::EVENT_LBUTTONDOWN)
    {
      RCLCPP_DEBUG(node->get_logger(), "Left mouse clicked at: (%d, %d)", x, y);
      
      // Get depth value at clicked position if image is available
      if (!node->current_depth_raw_.empty() && y >= 0 && y < node->current_depth_raw_.rows && 
          x >= 0 && x < node->current_depth_raw_.cols)
      {
        int depth_type = node->current_depth_raw_.type();
        if (depth_type == CV_16U)
        {
          uint16_t depth_value = node->current_depth_raw_.at<uint16_t>(y, x);
          RCLCPP_INFO(node->get_logger(), 
            "Depth at (%d, %d): %u mm (%.3f meters)", x, y, depth_value, depth_value / 1000.0f);
        }
        else if (depth_type == CV_16S)
        {
          int16_t depth_value = node->current_depth_raw_.at<int16_t>(y, x);
          RCLCPP_INFO(node->get_logger(), 
            "Depth at (%d, %d): %d mm (%.3f meters)", x, y, depth_value, depth_value / 1000.0f);
        }
        else if (depth_type == CV_32F)
        {
          float depth_value = node->current_depth_raw_.at<float>(y, x);
          RCLCPP_INFO(node->get_logger(), 
            "Depth at (%d, %d): %.3f meters", x, y, depth_value);
        }
      }
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
      RCLCPP_DEBUG(node->get_logger(), "Right mouse clicked at: (%d, %d)", x, y);
    }
    else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON))
    {
      RCLCPP_DEBUG(node->get_logger(), "Mouse dragging at: (%d, %d)", x, y);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CompressedDepthImageViewerNode>());
  rclcpp::shutdown();
  return 0;
}