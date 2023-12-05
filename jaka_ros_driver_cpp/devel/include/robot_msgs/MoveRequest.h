// Generated by gencpp from file robot_msgs/MoveRequest.msg
// DO NOT EDIT!


#ifndef ROBOT_MSGS_MESSAGE_MOVEREQUEST_H
#define ROBOT_MSGS_MESSAGE_MOVEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robot_msgs
{
template <class ContainerAllocator>
struct MoveRequest_
{
  typedef MoveRequest_<ContainerAllocator> Type;

  MoveRequest_()
    : pose()
    , has_ref(false)
    , ref_joint()
    , mvvelo(0.0)
    , mvacc(0.0)
    , mvtime(0.0)
    , mvradii(0.0)
    , coord_mode(0)
    , index(0)  {
    }
  MoveRequest_(const ContainerAllocator& _alloc)
    : pose(_alloc)
    , has_ref(false)
    , ref_joint(_alloc)
    , mvvelo(0.0)
    , mvacc(0.0)
    , mvtime(0.0)
    , mvradii(0.0)
    , coord_mode(0)
    , index(0)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _pose_type;
  _pose_type pose;

   typedef uint8_t _has_ref_type;
  _has_ref_type has_ref;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _ref_joint_type;
  _ref_joint_type ref_joint;

   typedef float _mvvelo_type;
  _mvvelo_type mvvelo;

   typedef float _mvacc_type;
  _mvacc_type mvacc;

   typedef float _mvtime_type;
  _mvtime_type mvtime;

   typedef float _mvradii_type;
  _mvradii_type mvradii;

   typedef int16_t _coord_mode_type;
  _coord_mode_type coord_mode;

   typedef int16_t _index_type;
  _index_type index;





  typedef boost::shared_ptr< ::robot_msgs::MoveRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_msgs::MoveRequest_<ContainerAllocator> const> ConstPtr;

}; // struct MoveRequest_

typedef ::robot_msgs::MoveRequest_<std::allocator<void> > MoveRequest;

typedef boost::shared_ptr< ::robot_msgs::MoveRequest > MoveRequestPtr;
typedef boost::shared_ptr< ::robot_msgs::MoveRequest const> MoveRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robot_msgs::MoveRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robot_msgs::MoveRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robot_msgs::MoveRequest_<ContainerAllocator1> & lhs, const ::robot_msgs::MoveRequest_<ContainerAllocator2> & rhs)
{
  return lhs.pose == rhs.pose &&
    lhs.has_ref == rhs.has_ref &&
    lhs.ref_joint == rhs.ref_joint &&
    lhs.mvvelo == rhs.mvvelo &&
    lhs.mvacc == rhs.mvacc &&
    lhs.mvtime == rhs.mvtime &&
    lhs.mvradii == rhs.mvradii &&
    lhs.coord_mode == rhs.coord_mode &&
    lhs.index == rhs.index;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robot_msgs::MoveRequest_<ContainerAllocator1> & lhs, const ::robot_msgs::MoveRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robot_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::MoveRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robot_msgs::MoveRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::MoveRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robot_msgs::MoveRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::MoveRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robot_msgs::MoveRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robot_msgs::MoveRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f6ccea1f6eaf1904632f66ac69537407";
  }

  static const char* value(const ::robot_msgs::MoveRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf6ccea1f6eaf1904ULL;
  static const uint64_t static_value2 = 0x632f66ac69537407ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_msgs::MoveRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robot_msgs/MoveRequest";
  }

  static const char* value(const ::robot_msgs::MoveRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robot_msgs::MoveRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# request: command specification for motion executions.\n"
"# Units:\n"
"#	joint space/angles: radian, radian/s and radian/s^2.\n"
"#	Cartesian space: mm, mm/s, and mm/s^2.\n"
"#	time: sec\n"
"\n"
"# pose： target coordinate. \n"
"#	For Joint Space target，pose dimention is the number of joints. element as each target joint position.\n"
"#	For Cartesian target: pose dimention is 6 for (x, y, z, roll, pitch, yaw)\n"
"float32[] pose\n"
"#Is there a reference solution \n"
"bool has_ref\n"
"#Send if there is, empty array if not \n"
"float32[] ref_joint\n"
"\n"
"# mvvelo: specified maximum velocity during execution. linear or angular velocity \n"
"float32 mvvelo\n"
"# mvacc: specified maximum acceleration during execution. linear or angular acceleration.\n"
"float32 mvacc\n"
"# mvtime: currently do not have any special meaning, please just give it 0.\n"
"float32 mvtime\n"
"# mvradii: this is special for move_ineb service, meaning the blending radius between 2 straight path trajectories, 0 for no blend.\n"
"float32 mvradii\n"
"int16 coord_mode\n"
"int16 index\n"
;
  }

  static const char* value(const ::robot_msgs::MoveRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robot_msgs::MoveRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose);
      stream.next(m.has_ref);
      stream.next(m.ref_joint);
      stream.next(m.mvvelo);
      stream.next(m.mvacc);
      stream.next(m.mvtime);
      stream.next(m.mvradii);
      stream.next(m.coord_mode);
      stream.next(m.index);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_msgs::MoveRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robot_msgs::MoveRequest_<ContainerAllocator>& v)
  {
    s << indent << "pose[]" << std::endl;
    for (size_t i = 0; i < v.pose.size(); ++i)
    {
      s << indent << "  pose[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.pose[i]);
    }
    s << indent << "has_ref: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.has_ref);
    s << indent << "ref_joint[]" << std::endl;
    for (size_t i = 0; i < v.ref_joint.size(); ++i)
    {
      s << indent << "  ref_joint[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.ref_joint[i]);
    }
    s << indent << "mvvelo: ";
    Printer<float>::stream(s, indent + "  ", v.mvvelo);
    s << indent << "mvacc: ";
    Printer<float>::stream(s, indent + "  ", v.mvacc);
    s << indent << "mvtime: ";
    Printer<float>::stream(s, indent + "  ", v.mvtime);
    s << indent << "mvradii: ";
    Printer<float>::stream(s, indent + "  ", v.mvradii);
    s << indent << "coord_mode: ";
    Printer<int16_t>::stream(s, indent + "  ", v.coord_mode);
    s << indent << "index: ";
    Printer<int16_t>::stream(s, indent + "  ", v.index);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOT_MSGS_MESSAGE_MOVEREQUEST_H
