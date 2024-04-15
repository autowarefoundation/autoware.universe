
#pragma once

#include <rs_driver/driver/input/input_sock.hpp>

namespace robosense
{
namespace lidar
{

class InputSockJumbo : public InputSock
{
public:

  InputSockJumbo(const RSInputParam& input_param)
    : InputSock(input_param)
  {
    pkt_buf_len_ = IP_LEN;
  }
};

}  // namespace lidar
}  // namespace robosense
