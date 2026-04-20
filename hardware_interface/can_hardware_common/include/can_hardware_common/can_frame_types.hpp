#ifndef CAN_HARDWARE_COMMON__CAN_FRAME_TYPES_HPP_
#define CAN_HARDWARE_COMMON__CAN_FRAME_TYPES_HPP_

#include <PCANBasic.h>

namespace can_hardware_common
{

using TxFrame = TPCANMsg;
using RxFrame = TPCANMsg;

}  // namespace can_hardware_common

#endif  // CAN_HARDWARE_COMMON__CAN_FRAME_TYPES_HPP_
