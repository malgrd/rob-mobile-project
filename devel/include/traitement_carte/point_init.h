// Generated by gencpp from file traitement_carte/point_init.msg
// DO NOT EDIT!


#ifndef TRAITEMENT_CARTE_MESSAGE_POINT_INIT_H
#define TRAITEMENT_CARTE_MESSAGE_POINT_INIT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace traitement_carte
{
template <class ContainerAllocator>
struct point_init_
{
  typedef point_init_<ContainerAllocator> Type;

  point_init_()
    : point_init()  {
    }
  point_init_(const ContainerAllocator& _alloc)
    : point_init(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _point_init_type;
  _point_init_type point_init;




  typedef boost::shared_ptr< ::traitement_carte::point_init_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::traitement_carte::point_init_<ContainerAllocator> const> ConstPtr;

}; // struct point_init_

typedef ::traitement_carte::point_init_<std::allocator<void> > point_init;

typedef boost::shared_ptr< ::traitement_carte::point_init > point_initPtr;
typedef boost::shared_ptr< ::traitement_carte::point_init const> point_initConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::traitement_carte::point_init_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::traitement_carte::point_init_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace traitement_carte

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'traitement_carte': ['/home/marion/catkin_ws/src/traitement_carte/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::traitement_carte::point_init_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traitement_carte::point_init_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::traitement_carte::point_init_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::traitement_carte::point_init_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traitement_carte::point_init_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traitement_carte::point_init_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::traitement_carte::point_init_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b1418312b1601ee4616c36827ab2854a";
  }

  static const char* value(const ::traitement_carte::point_init_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb1418312b1601ee4ULL;
  static const uint64_t static_value2 = 0x616c36827ab2854aULL;
};

template<class ContainerAllocator>
struct DataType< ::traitement_carte::point_init_<ContainerAllocator> >
{
  static const char* value()
  {
    return "traitement_carte/point_init";
  }

  static const char* value(const ::traitement_carte::point_init_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::traitement_carte::point_init_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point point_init\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::traitement_carte::point_init_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::traitement_carte::point_init_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.point_init);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct point_init_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::traitement_carte::point_init_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::traitement_carte::point_init_<ContainerAllocator>& v)
  {
    s << indent << "point_init: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.point_init);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRAITEMENT_CARTE_MESSAGE_POINT_INIT_H
