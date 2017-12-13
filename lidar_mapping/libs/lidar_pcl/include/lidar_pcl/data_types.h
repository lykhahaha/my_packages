#ifndef DATA_TYPES_H_
#define DATA_TYPES_H_

#include <fstream>

/*  
  Defining structures for unordered map hashing style for mapping 
*/

struct Pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  Pose():
      x(0.), y(0.), z(0.), roll(0.), pitch(0.), yaw(0.)
  {};

  Pose(const Pose &_p):
        x(_p.x), y(_p.y), z(_p.z), roll(_p.roll), pitch(_p.pitch), yaw(_p.yaw)
  {};

  Pose(const double &_x, const double &_y, const double &_z, 
       const double &_roll, const double &_pitch, const double &_yaw):
       x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw)
  {};

  friend std::ostream& operator<<(std::ostream& os, const Pose& ret) 
  { 
    os << "tl=[" << ret.x << "," << ret.y << "," << ret.z << "], " 
       << "rot=[" << ret.roll << "," << ret.pitch << "," << ret.yaw << "]";  
      return os;  
  }
};

struct Vel
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  Vel():
      x(0.), y(0.), z(0.), roll(0.), pitch(0.), yaw(0.)
  {};

  Vel(const Vel &_v):
      x(_v.x), y(_v.y), z(_v.z), roll(_v.roll), pitch(_v.pitch), yaw(_v.yaw)
  {};

  Vel(const double &_x, const double &_y, const double &_z, 
      const double &_roll, const double &_pitch, const double &_yaw):
      x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw)
  {};

  friend std::ostream& operator<<(std::ostream& os, const Vel& ret) 
  { 
    os << "tl=[" << ret.x << "," << ret.y << "," << ret.z << "], " 
       << "rot=[" << ret.roll << "," << ret.pitch << "," << ret.yaw << "]";  
      return os;  
  }
};

struct Accel
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  Accel():
      x(0.), y(0.), z(0.), roll(0.), pitch(0.), yaw(0.)
  {};

  Accel(const double &_x, const double &_y, const double &_z, 
        const double &_roll, const double &_pitch, const double &_yaw):
        x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw)
  {};

  friend std::ostream& operator<<(std::ostream& os, const Accel& ret) 
  { 
    os << "tl=[" << ret.x << "," << ret.y << "," << ret.z << "], " 
       << "rot=[" << ret.roll << "," << ret.pitch << "," << ret.yaw << "]";  
      return os;  
  }
};

struct Key
{
  int x;
  int y;
};

bool operator==(const Key& lhs, const Key& rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator!=(const Key& lhs, const Key& rhs)
{
  return lhs.x != rhs.x || lhs.y != rhs.y;
}

namespace std
{
  template <>
  struct hash<Key> // custom hashing function for Key<(x,y)>
  {
    std::size_t operator()(const Key& k) const
    {
      return hash<int>()(k.x) ^ (hash<int>()(k.y) << 1);
    }
  };
}

#endif // DATA_TYPES_H_