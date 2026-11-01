"""Patch OpenVINS' ROS 2 node so N mono cameras with a shared stamp enter the filter as ONE measurement.

Upstream queues each camera image as its own measurement and feeds them one by one, interleaved with IMU
callbacks. With 4 cameras the ROS 2 executor's callback order decides which images of a frame are fed
before the filter moves on (late ones are dropped as out of order), so the same bag gave different
trajectories (1.3 % vs 8.3 % ATE, rarely a crash). Here images are collected per timestamp and pushed as a
single CameraData holding every camera (OpenVINS tracks multi-image messages as parallel monos), so the
processing order is fixed. Frames still incomplete when a newer frame is complete are pushed as they are.
"""

import sys
from pathlib import Path

src = Path(sys.argv[1])
s = src.read_text()

old_push = """  // append it to our queue of images
  std::lock_guard<std::mutex> lck(camera_queue_mtx);
  camera_queue.push_back(message);
  std::sort(camera_queue.begin(), camera_queue.end());
}

void ROS2Visualizer::callback_stereo("""
new_push = """  // Collect this frame's images from all cameras; queue the frame as one measurement once complete.
  std::lock_guard<std::mutex> lck(camera_queue_mtx);
  static std::map<double, ov_core::CameraData> pending;
  auto &frame = pending[message.timestamp];
  frame.timestamp = message.timestamp;
  frame.sensor_ids.push_back(cam_id0);
  frame.images.push_back(message.images.at(0));
  frame.masks.push_back(message.masks.at(0));
  const size_t n_cams = (size_t)_app->get_params().state_options.num_cameras;
  if (frame.sensor_ids.size() < n_cams)
    return;
  // Complete: also flush older frames that can no longer complete, in time order.
  for (auto it = pending.begin(); it != pending.end() && it->first <= message.timestamp;) {
    ov_core::CameraData out;
    out.timestamp = it->second.timestamp;
    std::vector<size_t> order(it->second.sensor_ids.size());
    for (size_t k = 0; k < order.size(); k++) order[k] = k;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return it->second.sensor_ids[a] < it->second.sensor_ids[b]; });
    for (size_t k : order) {
      out.sensor_ids.push_back(it->second.sensor_ids[k]);
      out.images.push_back(it->second.images[k]);
      out.masks.push_back(it->second.masks[k]);
    }
    camera_queue.push_back(out);
    it = pending.erase(it);
  }
  std::sort(camera_queue.begin(), camera_queue.end());
}

void ROS2Visualizer::callback_stereo("""
assert s.count(old_push) == 1, "push pattern not found"
s = s.replace(old_push, new_push)

# The update thread waited for one queued message per camera; each message now carries all cameras.
old_wait = "    if (unique_cam_ids.size() == num_unique_cameras) {"
new_wait = "    if (!camera_queue.empty() && (unique_cam_ids.size() == num_unique_cameras || params.state_options.num_cameras > 2)) {"
assert s.count(old_wait) == 1, "wait pattern not found"
s = s.replace(old_wait, new_wait)
src.write_text(s)
print("patched", src)
