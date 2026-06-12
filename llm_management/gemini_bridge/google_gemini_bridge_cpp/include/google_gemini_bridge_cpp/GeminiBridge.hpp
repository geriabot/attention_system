#ifndef GOOGLE_GEMINI_BRIDGE_CPP__GEMINIBRIDGE_HPP_
#define GOOGLE_GEMINI_BRIDGE_CPP__GEMINIBRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "gemini_bridge_interfaces/srv/get_gemini_response.hpp"

#include "opencv2/opencv.hpp"

#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace gemini_testing
{

using json = nlohmann::json;
using namespace std::chrono_literals;

class GeminiBridge : public rclcpp::Node
{
public:
  GeminiBridge();

  bool init_gemini_client_service();

  const std::string BRIDGE_ERROR = std::string("__ERROR__");

private:
  rclcpp::Service<gemini_bridge_interfaces::srv::GetGeminiResponse>::SharedPtr service_;

  void ask_gemini(
    const gemini_bridge_interfaces::srv::GetGeminiResponse::Request::SharedPtr request,
    gemini_bridge_interfaces::srv::GetGeminiResponse::Response::SharedPtr response);

  static size_t http_perform_callback(void *ptr, size_t size, size_t nmemb, void *userdata);
  
  std::string make_prompt_request(const std::string& prompt,
                                  bool img_is_needed,
                                  const std::string& img_encoded,
                                  const std::string& output_json_properties);

  std::string cvmat_to_jpeg_base64(const cv::Mat& image, int quality);
  std::string get_http_json_text(json http_json);

  std::string api_key_;

  std::string request_endpoint_;

  bool waiting_http_ = false;

  int images_uploaded_ = 0;
  
  std::ostringstream header_stream_;

  const int JPEG_ENCODING_QUALITY = 95;
  const std::string BASE_URL = "https://generativelanguage.googleapis.com";
  const std::string MIME_TYPE = "image/jpeg";
  const std::string IMG_NAME_PREFIX = "prompt_image_";
};

} // namespace gemini_testing

#endif // GOOGLE_GEMINI_BRIDGE_CPP__GEMINIBRIDGE_HPP_
