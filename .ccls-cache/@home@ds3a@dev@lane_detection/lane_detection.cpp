#include <iostream>
#include "lane_extension.hpp"

const int image_size = 320;
const int window_length = 80;

int main() {
  int num_masks = image_size/window_length;
  cv::Mat image = cv::imread("../imgs/best_case_scenario.png");
  // cv::Mat image = cv::imread("../imgs/2nd_best_scenario.png");
  // cv::Mat image = cv::imread("../imgs/worst_case_scenario.png");

  // Check if image was successfully read
  if (image.empty()) {
    std::cout << "Error: Could not read image" << std::endl;
    return -1;
  }

  std::vector<cv::Mat> masks(num_masks);
  masks = lane_extension::generate_sliding_masks(image_size, window_length);

  std::cout << "\ngot " << lane_extension::extend_lanes(image, num_masks, masks).size() << " images\n";

  return 0;
}
