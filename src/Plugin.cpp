#include <mujoco/mjplugin.h>
#include "PosePublisher.h"

namespace MujocoRosUtils
{

mjPLUGIN_LIB_INIT
{
  PosePublisher::RegisterPlugin();
}

} //