#ifndef __SERVEROBORIO_H__

#include <mutex>

struct visionData {
  double pitch = 0.0;
  double yaw = 0.0;
  double distance = 0.0;
};

void ServeRoboRIOUtil(std::mutex&, visionData&);

#endif
