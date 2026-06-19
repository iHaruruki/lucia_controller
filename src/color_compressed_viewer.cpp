#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <map>
#include <algorithm>

class CompressedImageViewerNode : public rclcpp::Node
{
public:
  CompressedImageViewerNode() : Node("compressed_image_viewer_node")
  {
    // Declare parameters
    this->declare_parameter<int>("window_width", 640);
    this->declare_parameter<int>("window_height", 480);
    this->declare_parameter<int>("window_type", cv::WINDOW_NORMAL);
    this->declare_parameter<std::string>("window_name", "Color Image Viwer");
    this->declare_parameter<std::string>("image_format", "auto");  // auto, jpeg, png, webp, bmp
    this->declare_parameter<bool>("display_info", false);
    this->declare_parameter<bool>("record_frames", false);
    this->declare_parameter<std::string>("record_path", "/tmp/frames/");

    // Get parameter values
    window_width_ = this->get_parameter("window_width").as_int();
    window_height_ = this->get_parameter("window_height").as_int();
    window_type_ = this->get_parameter("window_type").as_int();
    window_name_ = this->get_parameter("window_name").as_string();
    image_format_ = this->get_parameter("image_format").as_string();
    display_info_ = this->get_parameter("display_info").as_bool();
    record_frames_ = this->get_parameter("record_frames").as_bool();
    record_path_ = this->get_parameter("record_path").as_string();

    // Initialize format mapping
    initializeFormatMap();

    RCLCPP_INFO(this->get_logger(), 
      "FLIR Viewer initialized - Window: %s, Size: %dx%d, Type: %d",
      window_name_.c_str(), window_width_, window_height_, window_type_);
    RCLCPP_INFO(this->get_logger(), 
      "Format: %s, Display Info: %s, Record: %s",
      image_format_.c_str(), 
      display_info_ ? "true" : "false", record_frames_ ? "true" : "false");

    // Create subscription to compressed image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/color/image_raw/compressed",
      rclcpp::SensorDataQoS(),
      std::bind(&CompressedImageViewerNode::image_callback, this, std::placeholders::_1));

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

  ~CompressedImageViewerNode()
  {
    cv::destroyAllWindows();
  }

private:
  int window_width_;
  int window_height_;
  int window_type_;
  std::string window_name_;
  std::string image_format_;
  bool display_info_;
  bool record_frames_;
  std::string record_path_;
  cv::Mat current_image_;
  std::map<std::string, int> format_map_;
  int frame_count_ = 0;

  void initializeFormatMap()
  {
    format_map_["jpeg"] = cv::IMREAD_COLOR;
    format_map_["jpg"] = cv::IMREAD_COLOR;
    format_map_["png"] = cv::IMREAD_COLOR;
    format_map_["webp"] = cv::IMREAD_COLOR;
    format_map_["bmp"] = cv::IMREAD_COLOR;
    format_map_["tiff"] = cv::IMREAD_COLOR;
    format_map_["auto"] = cv::IMREAD_COLOR;
  }

  int getDecodeFlag(const std::string& format)
  {
    std::string fmt = format;
    // Convert to lowercase
    std::transform(fmt.begin(), fmt.end(), fmt.begin(), ::tolower);

    if (format_map_.find(fmt) != format_map_.end())
    {
      return format_map_[fmt];
    }
    // Default to color image
    return cv::IMREAD_COLOR;
  }

  cv::Mat decodeImage(const sensor_msgs::msg::CompressedImage::SharedPtr& msg)
  {
    try
    {
      // Determine decoding flag based on format parameter or message format
      int decode_flag = cv::IMREAD_COLOR;
      
      if (image_format_ != "auto")
      {
        decode_flag = getDecodeFlag(image_format_);
      }
      else
      {
        // Try to detect format from message format field
        decode_flag = getDecodeFlag(msg->format);
      }

      // Create Mat from vector data
      cv::Mat compressed_mat(1, msg->data.size(), CV_8U, 
                            const_cast<uint8_t*>(msg->data.data()), cv::Mat::AUTO_STEP);

      // Decode image
      cv::Mat image = cv::imdecode(compressed_mat, decode_flag);

      if (image.empty())
      {
        RCLCPP_WARN(this->get_logger(), 
          "Failed to decode compressed image. Format: %s, Data size: %zu bytes",
          msg->format.c_str(), msg->data.size());
        return cv::Mat();
      }

      return image;
    }
    catch (const cv::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "OpenCV error decoding image: %s", e.what());
      return cv::Mat();
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error decoding image: %s", e.what());
      return cv::Mat();
    }
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
    info_lines.push_back("Size: " + std::to_string(image.cols) + "x" + std::to_string(image.rows));
    info_lines.push_back("Format: " + msg->format);
    info_lines.push_back("Compressed Size: " + std::to_string(msg->data.size()) + " bytes");

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
      std::string filename = record_path_ + "frame_" + 
                           std::to_string(frame_count_) + ".png";
      bool success = cv::imwrite(filename, image);
      
      if (!success)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to save frame to %s", filename.c_str());
      }
      else if (frame_count_ % 30 == 0)
      {
        RCLCPP_DEBUG(this->get_logger(), "Saved frame to %s", filename.c_str());
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error recording frame: %s", e.what());
    }
  }

  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    try
    {
      // Decode compressed image
      cv::Mat image = decodeImage(msg);

      if (image.empty())
      {
        return;
      }

      // Store current image
      current_image_ = image.clone();
      frame_count_++;

      // Draw info on image if enabled
      drawImageInfo(image, msg);

      // Record frame if enabled
      recordFrame(image);

      // Display image
      cv::imshow(window_name_, image);

      // Process key press (ESC to exit, 'S' to save, 'R' to toggle recording)
      int key = cv::waitKey(1);
      if (key == 27)  // ESC key
      {
        RCLCPP_INFO(this->get_logger(), "ESC key pressed, shutting down...");
        rclcpp::shutdown();
      }
      else if (key == 's' || key == 'S')  // Save current frame
      {
        std::string filename = "/tmp/flir_frame_" + 
                             std::to_string(frame_count_) + ".png";
        cv::imwrite(filename, current_image_);
        RCLCPP_INFO(this->get_logger(), "Saved frame to %s", filename.c_str());
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

      // Log frame info occasionally
      if (frame_count_ % 30 == 0)
      {
        RCLCPP_DEBUG(this->get_logger(), 
          "Frame %d: %dx%d, format: %s, data size: %zu bytes",
          frame_count_, image.cols, image.rows, msg->format.c_str(), msg->data.size());
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
    }
  }

  static void mouse_callback(int event, int x, int y, int flags, void* userdata)
  {
    CompressedImageViewerNode* node = static_cast<CompressedImageViewerNode*>(userdata);
    
    if (event == cv::EVENT_LBUTTONDOWN)
    {
      RCLCPP_DEBUG(node->get_logger(), "Left mouse clicked at: (%d, %d)", x, y);
      
      // Get pixel value at clicked position if image is available
      if (!node->current_image_.empty() && y >= 0 && y < node->current_image_.rows && 
          x >= 0 && x < node->current_image_.cols)
      {
        cv::Vec3b pixel = node->current_image_.at<cv::Vec3b>(y, x);
        RCLCPP_INFO(node->get_logger(), 
          "Pixel at (%d, %d): BGR(%d, %d, %d)", x, y, pixel[0], pixel[1], pixel[2]);
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
  rclcpp::spin(std::make_shared<CompressedImageViewerNode>());
  rclcpp::shutdown();
  return 0;
}