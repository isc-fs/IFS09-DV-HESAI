/*
 * File: isc_cloud.hpp
 * Description: ISC lean PointCloud2 layout for the ATX (ISC Formula Student fork)
 *
 * "isc" point layout, 20 bytes, every field naturally aligned:
 *   x f32 @0 | y f32 @4 | z f32 @8 | intensity u8 @12 | confidence u8 @13 | ring u16 @14 | time f32 @16
 * time is seconds relative to header.stamp (Velodyne-style), so deskew tools read it as-is.
 */

#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace isc
{

enum class PointFormat { kHesai, kIsc };
enum class StampSource { kFirstPacket, kLastPacket };

struct CloudOptions
{
  PointFormat format = PointFormat::kHesai;
  StampSource stamp = StampSource::kFirstPacket;
  bool drop_zero_points = false;
};

#pragma pack(push, 1)
struct IscPoint
{
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint8_t confidence;
  uint16_t ring;
  float time;
};
#pragma pack(pop)
static_assert(sizeof(IscPoint) == 20, "isc point layout must stay 20 bytes");

inline bool ParsePointFormat(const std::string& s, PointFormat& out)
{
  if (s == "hesai") { out = PointFormat::kHesai; return true; }
  if (s == "isc") { out = PointFormat::kIsc; return true; }
  return false;
}

inline bool ParseStampSource(const std::string& s, StampSource& out)
{
  if (s == "first_packet") { out = StampSource::kFirstPacket; return true; }
  if (s == "last_packet") { out = StampSource::kLastPacket; return true; }
  return false;
}

inline void SetStamp(sensor_msgs::msg::PointCloud2& msg, double t)
{
  msg.header.stamp.sec = static_cast<int32_t>(std::floor(t));
  msg.header.stamp.nanosec = static_cast<uint32_t>(std::round((t - std::floor(t)) * 1e9));
  if (msg.header.stamp.nanosec >= 1000000000u) {
    msg.header.stamp.sec += 1;
    msg.header.stamp.nanosec -= 1000000000u;
  }
}

inline void AddField(sensor_msgs::msg::PointCloud2& msg, const char* name, uint8_t type, uint32_t offset)
{
  sensor_msgs::msg::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = type;
  f.count = 1;
  msg.fields.push_back(f);
}

// Fills msg (fields, data, size, stamp) with the isc layout. With rows > 0 the grid is kept
// organized (height = rows) and zero points are never dropped, so indices stay meaningful.
template <typename PointT>
void FillIscCloud(sensor_msgs::msg::PointCloud2& msg, const PointT* pts, uint32_t n,
                  double stamp, bool drop_zero_points, uint32_t rows = 0)
{
  using sensor_msgs::msg::PointField;
  msg.fields.clear();
  AddField(msg, "x", PointField::FLOAT32, offsetof(IscPoint, x));
  AddField(msg, "y", PointField::FLOAT32, offsetof(IscPoint, y));
  AddField(msg, "z", PointField::FLOAT32, offsetof(IscPoint, z));
  AddField(msg, "intensity", PointField::UINT8, offsetof(IscPoint, intensity));
  AddField(msg, "confidence", PointField::UINT8, offsetof(IscPoint, confidence));
  AddField(msg, "ring", PointField::UINT16, offsetof(IscPoint, ring));
  AddField(msg, "time", PointField::FLOAT32, offsetof(IscPoint, time));
  msg.point_step = sizeof(IscPoint);
  msg.is_bigendian = false;
  SetStamp(msg, stamp);

  const bool organized = rows > 0 && n % rows == 0;
  const bool drop = drop_zero_points && !organized;
  msg.data.resize(static_cast<size_t>(n) * sizeof(IscPoint));
  auto* out = reinterpret_cast<IscPoint*>(msg.data.data());
  uint32_t kept = 0;
  for (uint32_t i = 0; i < n; i++) {
    const PointT& p = pts[i];
    const bool empty = p.x == 0.f && p.y == 0.f && p.z == 0.f;
    if (drop && empty) continue;
    IscPoint q;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    q.intensity = p.intensity;
    q.confidence = p.confidence;
    q.ring = p.ring;
    // Empty remake slots have timestamp 0; keep their time at 0 instead of -stamp.
    q.time = (p.timestamp > 0.0) ? static_cast<float>(p.timestamp - stamp) : 0.f;
    std::memcpy(out + kept, &q, sizeof(IscPoint));
    kept++;
  }
  msg.data.resize(static_cast<size_t>(kept) * sizeof(IscPoint));
  msg.height = organized ? rows : 1;
  msg.width = organized ? kept / rows : kept;
  msg.row_step = msg.width * msg.point_step;
  msg.is_dense = !organized && drop;
}

}  // namespace isc
