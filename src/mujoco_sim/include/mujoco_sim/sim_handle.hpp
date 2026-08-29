#ifndef MUJOCO_SIM_SIM_HANDLE_HPP_
#define MUJOCO_SIM_SIM_HANDLE_HPP_

#include <mutex>
#include <memory>
#include <mujoco/mujoco.h>

/**
 * @brief Thread-safe shared handle holding MuJoCo physics state
 * Enables safe concurrent access between physics stepping (500 Hz)
 * and camera rendering (30 Hz) without memory data races.
 */
struct SimHandle {
  mjModel* model{nullptr};
  mjData* data{nullptr};
  std::mutex mutex;

  ~SimHandle() {
    // Memory cleanup on destruction
    if (data) {
      mj_deleteData(data);
      data = nullptr;
    }
    if (model) {
      mj_deleteModel(model);
      model = nullptr;
    }
  }
};

using SimHandlePtr = std::shared_ptr<SimHandle>;

#endif  // MUJOCO_SIM_SIM_HANDLE_HPP_
