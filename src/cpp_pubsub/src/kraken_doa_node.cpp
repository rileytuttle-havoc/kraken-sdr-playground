#include <chrono>
#include <memory>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/doa.hpp"     // CHANGE
#include <curl/curl.h>
#include <iostream>

using namespace std::chrono_literals;

size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp)
{
  size_t totalSize = size * nmemb;
  std::string* s = static_cast<std::string*>(userp);
  s->append(static_cast<char*>(contents), totalSize);
  return totalSize;
}

class KrakenDOANode : public rclcpp::Node
{
public:
  // not using the Doa ros2 message
  // because I was hoping to inject a bit more type safety here
  // but that isn't done yet
  struct DoaPacket
  {
    uint64_t epoch_time;
    uint16_t max_doa_angle;
    float confidence;
    float rssi_power;
    float channel_freq;
    std::string array_arrangement;
    float latency_ms;
    std::string station_id;
    float latitude;
    float longitude;
    float gps_heading;
    float compass_heading;
    std::string main_heading_sensor_used;
    std::string R1;
    std::string R2;
    std::string R3;
    std::string R4;
    std::array<float, 360> output;
  };
  KrakenDOANode()
  : Node("kraken_doa_node"), count_(0)
  {
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Doa>("doa", 10);    // CHANGE
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&KrakenDOANode::timer_callback, this));
  }

private:
  std::string fetchCSV(const std::string& url)
  {
    CURL* curl = curl_easy_init();
    std::string response;

    if(curl) {
      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

      CURLcode res = curl_easy_perform(curl);
      if(res != CURLE_OK)
        std::cerr << "curl_easy_perform() failed: " 
                  << curl_easy_strerror(res) << std::endl;

      curl_easy_cleanup(curl);
    }

    return response;
  }

  DoaPacket parse_html(const std::string& html)
  {
    std::vector<std::string> elements;
    std::stringstream ss(html);
    std::string field;

    while(std::getline(ss, field, ','))
    {
      elements.push_back(field);
    }

    DoaPacket ret;
    ret.epoch_time = std::stoull(elements[0]);
    ret.max_doa_angle = static_cast<uint16_t>(std::stoul(elements[1]));
    ret.confidence = std::stof(elements[2]);
    ret.rssi_power = std::stof(elements[3]);
    ret.channel_freq = std::stof(elements[4]);
    ret.array_arrangement = elements[5];
    ret.latency_ms = std::stof(elements[6]);
    ret.station_id = elements[7];
    ret.latitude = std::stof(elements[8]);
    ret.longitude = std::stof(elements[9]);
    ret.gps_heading = std::stof(elements[10]);
    ret.compass_heading = std::stof(elements[11]);
    ret.main_heading_sensor_used = elements[12];
    ret.R1 = elements[13];
    ret.R2 = elements[14];
    ret.R3 = elements[15];
    ret.R4 = elements[16];
    // need to do more bounds checking first
    for(int i=0; i < 360; i++)
    {
      ret.output[i] = std::stof(elements[17+i]);
    }

    return ret;
  }
  void timer_callback()
  {
    auto message = tutorial_interfaces::msg::Doa();                               // CHANGE
    const auto html = fetchCSV("http://192.168.1.83:8081/DOA_value.html");
    const auto packet = parse_html(html);
    message.epoch_time = packet.epoch_time;
    message.max_doa_angle = packet.max_doa_angle;
    message.confidence = packet.confidence;
    message.rssi_power = packet.rssi_power;
    message.channel_freq = packet.channel_freq;
    message.array_arrangement = packet.array_arrangement;
    message.latency_ms = packet.latency_ms;
    message.station_id = packet.station_id;
    message.latitude = packet.latitude;
    message.longitude = packet.longitude;
    message.gps_heading = packet.gps_heading;
    message.compass_heading = packet.compass_heading;
    message.main_heading_sensor_used = packet.main_heading_sensor_used;
    message.r1 = packet.R1;
    message.r2 = packet.R2;
    message.r3 = packet.R3;
    message.r4 = packet.R4;
    message.doa_output = packet.output;
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::Doa>::SharedPtr publisher_;         // CHANGE
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KrakenDOANode>());
  rclcpp::shutdown();
  return 0;
}
