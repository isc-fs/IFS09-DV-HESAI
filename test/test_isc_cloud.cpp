#include <gtest/gtest.h>
#include <vector>
#include "lidar_types.h"
#include "isc_cloud.hpp"

using hesai::lidar::LidarPointXYZICRT;

namespace
{

LidarPointXYZICRT Pt(float x, float y, float z, uint8_t i, uint8_t c, uint16_t ring, double t)
{
  LidarPointXYZICRT p{};
  p.x = x; p.y = y; p.z = z; p.intensity = i; p.confidence = c; p.ring = ring; p.timestamp = t;
  return p;
}

const sensor_msgs::msg::PointField* Field(const sensor_msgs::msg::PointCloud2& m, const std::string& name)
{
  for (const auto& f : m.fields) if (f.name == name) return &f;
  return nullptr;
}

}  // namespace

TEST(IscCloud, LayoutIsAlignedAndTwentyBytes)
{
  std::vector<LidarPointXYZICRT> pts{Pt(1, 2, 3, 10, 2, 5, 100.0)};
  sensor_msgs::msg::PointCloud2 m;
  isc::FillIscCloud(m, pts.data(), 1, 100.0, false);
  EXPECT_EQ(m.point_step, 20u);
  for (const auto& f : m.fields) {
    uint32_t size = (f.datatype == sensor_msgs::msg::PointField::UINT8) ? 1 :
                    (f.datatype == sensor_msgs::msg::PointField::UINT16) ? 2 : 4;
    EXPECT_EQ(f.offset % size, 0u) << f.name;
  }
  ASSERT_NE(Field(m, "confidence"), nullptr);
  EXPECT_EQ(Field(m, "time")->datatype, sensor_msgs::msg::PointField::FLOAT32);
}

TEST(IscCloud, ValuesAndRelativeTime)
{
  std::vector<LidarPointXYZICRT> pts{Pt(1.5f, -2.f, 0.25f, 200, 3, 115, 1789911964.354976),
                                     Pt(4.f, 5.f, 6.f, 7, 1, 0, 1789911964.444976)};
  sensor_msgs::msg::PointCloud2 m;
  isc::FillIscCloud(m, pts.data(), 2, 1789911964.354976, false);
  ASSERT_EQ(m.width, 2u);
  EXPECT_EQ(m.height, 1u);
  EXPECT_EQ(m.header.stamp.sec, 1789911964);
  EXPECT_NEAR(m.header.stamp.nanosec, 354976000u, 1000);
  const auto* p = reinterpret_cast<const isc::IscPoint*>(m.data.data());
  EXPECT_FLOAT_EQ(p[0].x, 1.5f);
  EXPECT_FLOAT_EQ(p[0].y, -2.f);
  EXPECT_EQ(p[0].intensity, 200);
  EXPECT_EQ(p[0].confidence, 3);
  EXPECT_EQ(p[0].ring, 115);
  EXPECT_FLOAT_EQ(p[0].time, 0.f);
  // float32 relative time keeps sub-microsecond resolution, absolute float32 epochs would not
  EXPECT_NEAR(p[1].time, 0.09f, 1e-6);
}

TEST(IscCloud, DropsZeroPointsWhenUnorganized)
{
  std::vector<LidarPointXYZICRT> pts{Pt(1, 1, 1, 1, 0, 0, 10.0), Pt(0, 0, 0, 0, 0, 1, 10.0),
                                     Pt(2, 2, 2, 1, 0, 2, 10.0)};
  sensor_msgs::msg::PointCloud2 m;
  isc::FillIscCloud(m, pts.data(), 3, 10.0, true);
  EXPECT_EQ(m.width, 2u);
  EXPECT_EQ(m.data.size(), 2u * 20u);
  EXPECT_EQ(m.row_step, 40u);
  EXPECT_TRUE(m.is_dense);
  const auto* p = reinterpret_cast<const isc::IscPoint*>(m.data.data());
  EXPECT_EQ(p[1].ring, 2);
}

TEST(IscCloud, OrganizedGridKeepsEmptySlots)
{
  // 2 azimuth rows x 3 slots, one empty slot with timestamp 0 (cleared remake slot)
  std::vector<LidarPointXYZICRT> pts{Pt(1, 1, 1, 1, 0, 0, 10.0), LidarPointXYZICRT{}, Pt(1, 1, 1, 1, 0, 2, 10.0),
                                     Pt(1, 1, 1, 1, 0, 0, 10.1), Pt(1, 1, 1, 1, 0, 1, 10.1), Pt(1, 1, 1, 1, 0, 2, 10.1)};
  sensor_msgs::msg::PointCloud2 m;
  isc::FillIscCloud(m, pts.data(), 6, 10.0, true, 2);
  EXPECT_EQ(m.height, 2u);
  EXPECT_EQ(m.width, 3u);
  EXPECT_FALSE(m.is_dense);
  const auto* p = reinterpret_cast<const isc::IscPoint*>(m.data.data());
  EXPECT_FLOAT_EQ(p[1].time, 0.f);
  EXPECT_NEAR(p[4].time, 0.1f, 1e-5);
}

TEST(IscCloud, LastPacketStampGivesNegativeTimes)
{
  std::vector<LidarPointXYZICRT> pts{Pt(1, 1, 1, 1, 0, 0, 20.0), Pt(1, 1, 1, 1, 0, 0, 20.09)};
  sensor_msgs::msg::PointCloud2 m;
  isc::FillIscCloud(m, pts.data(), 2, 20.09, false);
  const auto* p = reinterpret_cast<const isc::IscPoint*>(m.data.data());
  EXPECT_NEAR(p[0].time, -0.09f, 1e-5);
  EXPECT_FLOAT_EQ(p[1].time, 0.f);
}

TEST(IscCloud, ParsesOptions)
{
  isc::PointFormat f;
  isc::StampSource s;
  EXPECT_TRUE(isc::ParsePointFormat("isc", f));
  EXPECT_EQ(f, isc::PointFormat::kIsc);
  EXPECT_FALSE(isc::ParsePointFormat("ISC", f));
  EXPECT_TRUE(isc::ParseStampSource("last_packet", s));
  EXPECT_EQ(s, isc::StampSource::kLastPacket);
  EXPECT_FALSE(isc::ParseStampSource("mid", s));
}
