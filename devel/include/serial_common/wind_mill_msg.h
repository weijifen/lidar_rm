// Generated by gencpp from file serial_common/wind_mill_msg.msg
// DO NOT EDIT!


#ifndef SERIAL_COMMON_MESSAGE_WIND_MILL_MSG_H
#define SERIAL_COMMON_MESSAGE_WIND_MILL_MSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace serial_common
{
template <class ContainerAllocator>
struct wind_mill_msg_
{
  typedef wind_mill_msg_<ContainerAllocator> Type;

  wind_mill_msg_()
    : horizonal(0.0)
    , vertical(0.0)  {
    }
  wind_mill_msg_(const ContainerAllocator& _alloc)
    : horizonal(0.0)
    , vertical(0.0)  {
  (void)_alloc;
    }



   typedef float _horizonal_type;
  _horizonal_type horizonal;

   typedef float _vertical_type;
  _vertical_type vertical;





  typedef boost::shared_ptr< ::serial_common::wind_mill_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serial_common::wind_mill_msg_<ContainerAllocator> const> ConstPtr;

}; // struct wind_mill_msg_

typedef ::serial_common::wind_mill_msg_<std::allocator<void> > wind_mill_msg;

typedef boost::shared_ptr< ::serial_common::wind_mill_msg > wind_mill_msgPtr;
typedef boost::shared_ptr< ::serial_common::wind_mill_msg const> wind_mill_msgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::serial_common::wind_mill_msg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::serial_common::wind_mill_msg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace serial_common

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {'serial_common': ['/home/zhanggang/PRLIDAR/catkin_ws1/src/serial_common/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::serial_common::wind_mill_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_common::wind_mill_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serial_common::wind_mill_msg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serial_common::wind_mill_msg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_common::wind_mill_msg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_common::wind_mill_msg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::serial_common::wind_mill_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cac8e6416acc3c4967ad84a4b158da62";
  }

  static const char* value(const ::serial_common::wind_mill_msg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcac8e6416acc3c49ULL;
  static const uint64_t static_value2 = 0x67ad84a4b158da62ULL;
};

template<class ContainerAllocator>
struct DataType< ::serial_common::wind_mill_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "serial_common/wind_mill_msg";
  }

  static const char* value(const ::serial_common::wind_mill_msg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::serial_common::wind_mill_msg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 horizonal\n\
float32 vertical\n\
";
  }

  static const char* value(const ::serial_common::wind_mill_msg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::serial_common::wind_mill_msg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.horizonal);
      stream.next(m.vertical);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct wind_mill_msg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::serial_common::wind_mill_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::serial_common::wind_mill_msg_<ContainerAllocator>& v)
  {
    s << indent << "horizonal: ";
    Printer<float>::stream(s, indent + "  ", v.horizonal);
    s << indent << "vertical: ";
    Printer<float>::stream(s, indent + "  ", v.vertical);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERIAL_COMMON_MESSAGE_WIND_MILL_MSG_H
