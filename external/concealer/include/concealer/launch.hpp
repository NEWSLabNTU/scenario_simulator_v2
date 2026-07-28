// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONCEALER__LAUNCH_HPP_
#define CONCEALER__LAUNCH_HPP_

#include <boost/algorithm/string.hpp>
#include <concealer/execute.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <system_error>
#include <type_traits>
#include <vector>

namespace concealer
{
template <typename Parameters>
auto ros2_launch(
  const std::string & package, const std::string & file, const Parameters & parameters)
{
  const auto argv = [&]() {
    auto argv = std::vector<std::string>();

    // Use play_launch if available (faster Rust parser, web UI, monitoring).
    // Falls back to ros2 launch if play_launch is not installed.
    const auto play_launch_path = dollar("which play_launch 2>/dev/null");
    const auto trimmed_path =
      boost::algorithm::replace_all_copy(play_launch_path, "\n", "");
    if (!trimmed_path.empty()) {
      argv.push_back(trimmed_path);
      argv.push_back("launch");
      // Web UI address for the Autoware this forks. It must differ from the outer
      // play_launch (default :8080), and from any other stack on this host -- a second
      // simulator, or a background AV's Autoware, would otherwise collide on the port and
      // fail to bind. Override with PLAY_LAUNCH_WEB_ADDR.
      const auto web_addr_env = std::getenv("PLAY_LAUNCH_WEB_ADDR");
      argv.push_back("--web-addr");
      argv.push_back(web_addr_env != nullptr ? std::string(web_addr_env) : "0.0.0.0:8082");
    } else {
      argv.push_back("python3");
      argv.push_back(boost::algorithm::replace_all_copy(dollar("which ros2"), "\n", ""));
      argv.push_back("launch");
    }

    argv.push_back(package);
    argv.push_back(file);

    for (const auto & parameter : parameters) {
      argv.push_back(parameter);
    }

    return argv;
  }();

  if (const auto process_id = fork(); process_id < 0) {
    throw std::system_error(errno, std::system_category());
  } else if (process_id == 0 and execute(argv) < 0) {
    std::cout << std::system_error(errno, std::system_category()).what() << std::endl;
    std::exit(EXIT_FAILURE);
  } else {
    return process_id;
  }
}
}  // namespace concealer

#endif  // CONCEALER__LAUNCH_HPP_
