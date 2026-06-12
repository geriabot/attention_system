#include "google_gemini_bridge_cpp/GeminiBridge.hpp"

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdlib>
#include <string>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Exception.hpp>
#include <curlpp/Options.hpp>

#include "base64.h"

namespace gemini_testing
{

using json = nlohmann::json;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

GeminiBridge::GeminiBridge()
: Node("gemini_bridge")
{
  this->declare_parameter("model", "gemini-2.5-flash-lite");

  std::string model;

  this->get_parameter("model", model);
  RCLCPP_INFO(this->get_logger(), "Model: %s", model.c_str());

  request_endpoint_ = BASE_URL + "/v1beta/models/" + model + ":generateContent";
}

bool
GeminiBridge::init_gemini_client_service()
{
  const char* api_key_value = std::getenv("GOOGLE_GEMINI_API_KEY");
  if (api_key_value == nullptr) {
    return false;
  }

  api_key_ = std::string(api_key_value);

  service_ = this->create_service<gemini_bridge_interfaces::srv::GetGeminiResponse>(
    "gemini_bridge_service",
    std::bind(&GeminiBridge::ask_gemini, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "Service ON!");

  return true;
}

void
GeminiBridge::ask_gemini(
  const gemini_bridge_interfaces::srv::GetGeminiResponse::Request::SharedPtr request,
  gemini_bridge_interfaces::srv::GetGeminiResponse::Response::SharedPtr response)
{
  RCLCPP_INFO(this->get_logger(), "PROMPT RECEIVED");
  std::cout << "-----------------\n";
  std::cout << request->prompt << std::endl;
  std::cout << "-----------------\n";

  try {
    cv::Mat img;
    if (request->uses_image) {
      cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(request->img, sensor_msgs::image_encodings::BGR8);
      img = img_ptr->image.clone();
    }

    std::string http_response;
    std::string img_buffer;

    RCLCPP_INFO(this->get_logger(), "KEY: %s", api_key_.c_str());

    if (request->uses_image) {
      http_response = make_prompt_request(request->prompt,
        false,
        img_buffer,
        request->output_fields_json);

    } else {
      http_response = make_prompt_request(request->prompt,
        false,
        img_buffer,
        request->output_fields_json);
    }


    std::cout << "RESPONSE:\n\n" << http_response << std::endl << std::endl;

    if (http_response == BRIDGE_ERROR) {
      response->gemini_response = BRIDGE_ERROR;
      return;
    }

    response->gemini_response = http_response;

    waiting_http_ = false;
  
  } catch (const curlpp::RuntimeError& e) {
    RCLCPP_ERROR(this->get_logger(), "ERROR:");
    std::cerr << "curlpp runtime error: " << e.what() << std::endl;
    response->gemini_response = BRIDGE_ERROR;

  } catch (const curlpp::LogicError& e) {
    RCLCPP_ERROR(this->get_logger(), "ERROR:");
    std::cerr << "curlpp logic error: " << e.what() << std::endl;
    response->gemini_response = BRIDGE_ERROR;
  
  } catch (const json::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "ERROR:");
    std::cerr << "JSON parse error: " << e.what() << std::endl;
    response->gemini_response = BRIDGE_ERROR;

  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "ERROR:");
    std::cerr << "Runtime error: " << e.what() << std::endl;
    response->gemini_response = BRIDGE_ERROR;
  }
}

std::string
GeminiBridge::get_http_json_text(json http_json)
{
  if (http_json.contains("candidates")) {
    for (const auto& candidate : http_json["candidates"]) {
      if (candidate.contains("content") && candidate["content"].contains("parts")) {
        for (const auto& part : candidate["content"]["parts"]) {
          if (part.contains("text")) {
            return part["text"];
          }
        }
      }
    }
  }

  return BRIDGE_ERROR;
}

std::string
GeminiBridge::cvmat_to_jpeg_base64(const cv::Mat& image, int quality)
{
  std::vector<unsigned char> buff;
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, quality};
  if (!cv::imencode(".jpeg", image, buff, params)) {
    throw std::runtime_error("Failed to encode image to JPEG.");
  }

  unsigned char const* uc_buff = buff.data();

  std::string encoded_img = base64_encode(uc_buff, buff.size());

  return encoded_img;
}

size_t
GeminiBridge::http_perform_callback(void *ptr, size_t size, size_t nmemb, void *userdata)
{
  auto *response = static_cast<std::ostringstream*>(userdata);
  size_t total_size = size * nmemb;
  response->write(static_cast<char*>(ptr), total_size);
  return total_size;
}

std::string
GeminiBridge::make_prompt_request(const std::string& prompt,
                                  bool img_is_needed,
                                  const std::string& img_encoded,
                                  const std::string& output_json_properties)
{
  RCLCPP_INFO(this->get_logger(), "URL (Endpoint): %s", request_endpoint_.c_str());

  json response_properties = json::parse(output_json_properties);
  
  std::cout << "----------------------\nPROPERTIES:\n";
  std::cout << response_properties.dump(2) << std::endl;
  std::cout << "----------------------\n";

  std::vector<std::string> property_ordering;
  for (json::iterator it = response_properties.begin(); it != response_properties.end(); ++it) {
    property_ordering.push_back(it.key());
  }

  json request_json;

  if (img_is_needed) {
    request_json = {
      {"contents", json::array({
        {
          {"parts", json::array({
            {{"text", prompt}},
            {
              {"inline_data",
                {
                  {"mime_type", "image/jpeg"},
                  {"data", img_encoded}
                }
              }
            }
          })}
        }
      })},
      {"generationConfig",
        {
          {"responseMimeType", "application/json"},
          {"responseSchema", 
            {
              {"type", "ARRAY"},
              {"items",
                {
                  {"type", "OBJECT"},
                  {"properties", response_properties},
                  {"propertyOrdering", property_ordering}
                }
              }
            }
          },
        }
      }
    };
  } else {
    request_json = {
      {"contents", json::array({
        {
          {"parts", json::array({
            {{"text", prompt}},
          })}
        }
      })},
      {"generationConfig",
        {
          {"responseMimeType", "application/json"},
          {"responseSchema", response_properties},
        }
      }
    };
  }

  RCLCPP_INFO(this->get_logger(), "PROMPT REQUEST JSON:");
  std::cout << request_json.dump(2) << std::endl;

  std::list<std::string> headers = {
      "x-goog-api-key: " + api_key_,
      "Content-Type: application/json"
  };

  curlpp::Easy request;
  request.setOpt(curlpp::Options::Url(request_endpoint_));
  request.setOpt(new curlpp::Options::HttpHeader(headers));
  request.setOpt(new curlpp::Options::PostFields(request_json.dump()));
  request.setOpt(new curlpp::Options::PostFieldSize(request_json.dump().size()));

  std::ostringstream response_stream;
  request.setOpt(new curlpp::Options::WriteStream(&response_stream));

  request.perform();

  RCLCPP_INFO(this->get_logger(), "GEMINI RESPONSE:");
  std::cout << response_stream.str() << std::endl;

  json response_json = json::parse(response_stream.str());
  if (!response_json.contains("candidates")) {
    RCLCPP_ERROR(this->get_logger(), "{make_prompt_request} No candidates received");
    return BRIDGE_ERROR;
  }

  std::string generated_text = get_http_json_text(response_json);

  return generated_text;
}

} // namespace gemini_testing
