// Copyright (c) 2022, Zhenpeng Ge (https://github.com/gezp)
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

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_components/component_manager.hpp"

namespace rclcpp_components
{
/// ComponentManagerIsolated uses dedicated single-threaded executors for each components.
template<typename ExecutorT = rclcpp::executors::SingleThreadedExecutor>
class ComponentManagerIsolated : public rclcpp_components::ComponentManager
{
  using rclcpp_components::ComponentManager::ComponentManager;

  struct DedicatedExecutorWrapper
  {
    std::shared_ptr<rclcpp::Executor> executor;
    std::thread thread;
  };

public:
  ~ComponentManagerIsolated()
  {
    if (node_wrappers_.size()) {
      for (auto & executor_wrapper : dedicated_executor_wrappers_) {
        executor_wrapper.second.executor->cancel();
        executor_wrapper.second.thread.join();
      }
      node_wrappers_.clear();
    }
  }

protected:
  /// Add component node to executor model, it's invoked in on_load_node()
  /**
   * \param node_id  node_id of loaded component node in node_wrappers_
   */
  void
  add_node_to_executor(uint64_t node_id) override
  {
    DedicatedExecutorWrapper executor_wrapper;
    auto exec = std::make_shared<ExecutorT>();
    exec->add_node(node_wrappers_[node_id].get_node_base_interface());
    executor_wrapper.executor = exec;
    executor_wrapper.thread = std::thread(
      [exec]() {
        exec->spin();
      });
    dedicated_executor_wrappers_[node_id] = std::move(executor_wrapper);
  }
  /// Remove component node from executor model, it's invoked in on_unload_node()
  /**
   * \param node_id  node_id of loaded component node in node_wrappers_
   */
  void
  remove_node_from_executor(uint64_t node_id) override
  {
    auto executor_wrapper = dedicated_executor_wrappers_.find(node_id);
    if (executor_wrapper != dedicated_executor_wrappers_.end()) {
      executor_wrapper->second.executor->cancel();
      executor_wrapper->second.thread.join();
      dedicated_executor_wrappers_.erase(executor_wrapper);
    }
  }

private:
  std::unordered_map<uint64_t, DedicatedExecutorWrapper> dedicated_executor_wrappers_;
};

}  // namespace rclcpp_components

int main(int argc, char * argv[])
{
  /// Component container with dedicated single-threaded executors for each components.
  rclcpp::init(argc, argv);
  // parse arguments
  bool use_multi_threaded_executor{false};
  std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  for (auto & arg : args) {
    if (arg == std::string("--use_multi_threaded_executor")) {
      use_multi_threaded_executor = true;
    }
  }
  // create executor and component manager
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  rclcpp::Node::SharedPtr node;
  if (use_multi_threaded_executor) {
    using ComponentManagerIsolated =
      rclcpp_components::ComponentManagerIsolated<rclcpp::executors::MultiThreadedExecutor>;
    node = std::make_shared<ComponentManagerIsolated>(exec);
  } else {
    using ComponentManagerIsolated =
      rclcpp_components::ComponentManagerIsolated<rclcpp::executors::SingleThreadedExecutor>;
    node = std::make_shared<ComponentManagerIsolated>(exec);
  }
  exec->add_node(node);
  exec->spin();
}