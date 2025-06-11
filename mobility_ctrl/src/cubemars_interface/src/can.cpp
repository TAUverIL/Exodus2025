#include "cubemars_interface/can.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

namespace cubemars_interface
{
bool CanSocket::connect(std::string can_itf, const std::vector<canid_t> & can_ids, canid_t can_mask)
{
  // open socket
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CubeMarsSystemHardware"),
      "Could not create socket");
    return false;
  }
  fprintf(stderr, ">>> Got Here!! <<<");
  // get CAN interface index
  struct ifreq ifr;
  strcpy(ifr.ifr_name, can_itf.c_str());
  ioctl(socket_, SIOCGIFINDEX, &ifr);

  // bind CAN interface
  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CubeMarsSystemHardware"),
      "Could not bind CAN interface");
      RCLCPP_ERROR(
      rclcpp::get_logger("CubeMarsSystemHardware"),
      "CAN mask = %d", can_mask);
    return false;
  }

  // filter CAN IDs
  // can_mask_ = can_mask;

  auto logger = rclcpp::get_logger("CubeMarsSystemHardware");

  can_mask_ = CAN_EFF_MASK | CAN_EFF_FLAG;
  struct can_filter rfilter[can_ids.size()];
  for (std::size_t i = 0; i < can_ids.size(); i++)
  {
    // rfilter[i].can_id = can_ids[i];
    // rfilter[i].can_mask = can_mask_;

    uint32_t id = can_ids[i] | CAN_EFF_FLAG;
    uint32_t mask = CAN_EFF_MASK | CAN_EFF_FLAG;
    rfilter[i].can_id = id;
    rfilter[i].can_mask = mask;
    RCLCPP_INFO(logger, "CAN filter results: can_id=0x%08X, can_mask=0x%08X", id, mask);
  }
  setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

  // write test message
  if (!write_message(0, NULL, 0))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CubeMarsSystemHardware"),
      "Test message failed");
    return false;
  }

  return true;
}

bool CanSocket::disconnect()
{
  if (close(socket_) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CubeMarsSystemHardware"),
      "Could not close CAN socket");
    return false;
  }
  return true;
}

bool CanSocket::read_nonblocking(std::uint32_t & id, std::uint8_t data[], std::uint8_t & len)
{
  struct can_frame frame;
  if (recv(socket_, &frame, sizeof(struct can_frame), MSG_DONTWAIT) < 0)
  {
    if (errno == EAGAIN)
    {
      // no message in buffer
    }
    else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("CubeMarsSystemHardware"),
        "Could not read CAN socket");
    }
    return false;
  }

  memcpy(data, frame.data, frame.len);
  id = frame.can_id & can_mask_;
  len = frame.len;

  return true;
}

bool CanSocket::write_message(std::uint32_t id, const std::uint8_t data[], std::uint8_t len)
{
  struct can_frame frame;
  frame.can_id = id | CAN_EFF_FLAG;
  frame.len = len;
  memcpy(frame.data, data, len);
  if (write(socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CubeMarsSystemHardware"),
      "Could not write message to CAN socket");
    return false;
  }
  return true;
}
}