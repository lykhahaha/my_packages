#ifndef _LIDAR_PCL_H_
#define _LIDAR_PCL_H_

#define PCL_NO_PRECOMPILE // to create a custom pcl point type

#include <pcl_conversions/pcl_conversions.h>

namespace lidar_pcl
{
  struct PointXYZIR
  {
    PCL_ADD_POINT4D; // add x,y,z member + padding
    union
    {
      struct
      {
        float intensity;
        uint16_t ring;
      };
      float data_c[4];
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    inline PointXYZIR()
    {
      x = 0.; y = 0.; z = 0.;
      intensity = 0.;
      ring = 0;
    }

    inline PointXYZIR(const lidar_pcl::PointXYZIR &p)
    {
      x = p.x; y = p.y; z = p.z;
      intensity = p.intensity;
      ring = p.ring;
    }

    inline PointXYZIR(float _x, float _y, float _z, float _intensity, uint16_t _ring)
    {
      x = _x; y = _y; z = _z;
      intensity = _intensity;
      ring = _ring;
    }

    inline PointXYZIR(const uint8_t * source)
    {
      memcpy(&x, source, 4);
      memcpy(&y, source + 4, 4);
      memcpy(&z, source + 8, 4);
      memcpy(&intensity, source + 16, 4);
      memcpy(&ring, source + 20, 4);
    }

    friend std::ostream& operator<<(std::ostream& os, const PointXYZIR& ret) 
    { 
      os << "(" << ret.x << "," << ret.y << "," << ret.z << "," 
         << ret.intensity << "," << ret.ring << ")" ;  
      return os;  
    } 
  }EIGEN_ALIGN16;

  inline float getYawAngleFromPtr(const uint8_t* data)
  {
    float _x, _y;
    memcpy(&_x, data, 4);
    memcpy(&_y, data + 4, 4);
    return std::atan2(_y, _x) * 180 / 3.14159265359; // degree value
  }

  inline float getYawAngle(float _x, float _y)
  {
    return std::atan2(_y, _x) * 180 / 3.14159265359; // degree value
  }

  inline float calculateMinAngleDist(float first, float second)
  {
    float difference = first - second;
    if(difference >= 180.0)
      return difference - 360.0;
    if(difference <= -180.0)
      return difference + 360.0;
    return difference;
  }

  template <typename PointT>
  void fromPCLPointCloud2Custom(const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud,
                                const pcl::MsgFieldMap& field_map)
  {
    // Copy info fields
    cloud.header   = msg.header;
    cloud.width    = msg.width;
    cloud.height   = msg.height;
    cloud.is_dense = msg.is_dense == 1;
     
    // Copy point data
    uint32_t num_points = msg.width * msg.height;
    cloud.points.resize(num_points);
    uint8_t* cloud_data = reinterpret_cast<uint8_t*>(&cloud.points[0]);
    
    // Get first point data
    float origin_angle = getYawAngleFromPtr(&msg.data[0]);
    bool half_circle_check = false;
    float stop_threshold = 45.0; // angle, in degree

    // memcpy each group of contiguous fields separately
    for (uint32_t row = 0; row < msg.height; ++row)
    {
      const uint8_t* row_data = &msg.data[row * msg.row_step];
      for (uint32_t col = 0; col < msg.width; ++col)
      {
        const uint8_t* msg_data = row_data + col * msg.point_step;

        // Get relative angle position of current point
        
        float current_relative_angle = calculateMinAngleDist(getYawAngleFromPtr(&msg_data[0]), 
                                                             origin_angle);

        // assuming CW (-) rotation (checked with all current models)
        if(!half_circle_check)
        {
          if(std::fabs(current_relative_angle) > 45.0)
            half_circle_check = true;
          BOOST_FOREACH (const pcl::detail::FieldMapping& mapping, field_map)
          {
            memcpy(cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
          }
          cloud_data += sizeof(PointT);
        }
        else // half_circle_check == true
        {
          if(std::fabs(current_relative_angle) > stop_threshold)
          {
            BOOST_FOREACH (const pcl::detail::FieldMapping& mapping, field_map)
            {
              memcpy(cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
            }
            cloud_data += sizeof(PointT);
          }
          else
          {
            // 360 degree has been traversed through, stop adding more points to cloud
            // Resize cloud to fit then exit
            cloud.width = col + 1;
            num_points = row * msg.width + (col + 1);
            cloud.points.resize(num_points);
            return; 
          }
        }
      }
    }
  }

  template <typename PointT>
  void fromROSMsg(const sensor_msgs::PointCloud2& cloud, pcl::PointCloud<PointT> &pcl_cloud)
  {
    ::pcl::PCLPointCloud2 pcl_pc2;
    ::pcl_conversions::toPCL(cloud, pcl_pc2);
    ::pcl::MsgFieldMap field_map;
    ::pcl::createMapping<PointT>(pcl_pc2.fields, field_map);
    fromPCLPointCloud2Custom(pcl_pc2, pcl_cloud, field_map);
  }
} // namespace lidar_pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_pcl::PointXYZIR,  // here we assume a XYZ + "intensity" + ring (as fields)
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (uint16_t, ring, ring)
)

#endif // _LIDAR_PCL_H_