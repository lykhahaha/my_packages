#ifndef STRUCT_TYPES_H_
#define STRUCT_TYPES_H_

/*  
  Defining Key structure for unordered map hashing style for mapping 
*/

struct Pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

struct Vel
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

struct Accel
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
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

#endif // STRUCT_TYPES_H_