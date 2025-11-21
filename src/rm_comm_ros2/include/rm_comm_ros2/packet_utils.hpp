#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace rm_comm_ros2 {

  // 将标准布局结构体序列化为 std::string（零终止不含在内）
  // 目标用于 64 字节包传输

  template <typename T> inline std::string to_bytes(const T& data) {
    static_assert(std::is_standard_layout<T>::value, "Type T must be a standard layout type.");
    std::vector<char> buffer(sizeof(T));
    std::memcpy(buffer.data(), &data, sizeof(T));
    return std::string(buffer.begin(), buffer.end());
  }

  // 从 std::string 反序列化为结构体，要求大小严格匹配

  template <typename T> inline bool from_bytes(const std::string& s, T& out) {
    static_assert(std::is_standard_layout<T>::value, "Type must be standard layout");
    if (s.size() != sizeof(T)) {
      return false;
    }
    std::memcpy(&out, s.data(), sizeof(T));
    return true;
  }

  // 从任意字节流中提取定长包（如 64 字节），使用帧头匹配与自定义校验器
  // validator: bool(const T&) 返回该包是否有效（例如检查 frame_tail 等）

  template <typename T, typename Validator>
  inline std::vector<T> extract_packets(const std::string& buffer, uint8_t frame_header,
                                        Validator validator) {
    static_assert(std::is_standard_layout<T>::value, "Type must be standard layout");
    std::vector<T> results;
    const size_t   packet_size = sizeof(T);
    if (buffer.size() < packet_size) {
      return results;
    }
    for (size_t i = 0; i + packet_size <= buffer.size();) {
      const uint8_t* ptr = reinterpret_cast<const uint8_t*>(buffer.data() + i);
      if (ptr[0] == frame_header) {
        T pkt;
        std::memcpy(&pkt, ptr, packet_size);
        if (validator(pkt)) {
          results.emplace_back(pkt);
          i += packet_size;
          continue;
        }
      }
      // 未对齐或校验失败，步进 1 字节继续搜索
      ++i;
    }
    return results;
  }

} // namespace rm_comm_ros2