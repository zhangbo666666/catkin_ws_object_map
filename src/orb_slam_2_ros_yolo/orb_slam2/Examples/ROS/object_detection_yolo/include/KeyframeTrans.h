// Generated by gencpp from file object_map_msgs/KeyframeTrans.msg
// DO NOT EDIT!


#ifndef OBJECT_MAP_MSGS_MESSAGE_KEYFRAMETRANS_H
#define OBJECT_MAP_MSGS_MESSAGE_KEYFRAMETRANS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>

namespace object_map_msgs
{
template <class ContainerAllocator>
struct KeyframeTrans_
{
  typedef KeyframeTrans_<ContainerAllocator> Type;

  KeyframeTrans_()
    : header()
    , keyframeTrans()  {
    }
  KeyframeTrans_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , keyframeTrans(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::std_msgs::Float64MultiArray_<ContainerAllocator>  _keyframeTrans_type;
  _keyframeTrans_type keyframeTrans;





  typedef boost::shared_ptr< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> const> ConstPtr;

}; // struct KeyframeTrans_

typedef ::object_map_msgs::KeyframeTrans_<std::allocator<void> > KeyframeTrans;

typedef boost::shared_ptr< ::object_map_msgs::KeyframeTrans > KeyframeTransPtr;
typedef boost::shared_ptr< ::object_map_msgs::KeyframeTrans const> KeyframeTransConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::object_map_msgs::KeyframeTrans_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::object_map_msgs::KeyframeTrans_<ContainerAllocator1> & lhs, const ::object_map_msgs::KeyframeTrans_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.keyframeTrans == rhs.keyframeTrans;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::object_map_msgs::KeyframeTrans_<ContainerAllocator1> & lhs, const ::object_map_msgs::KeyframeTrans_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace object_map_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7867742dbcaa5ee32577032d2a8da3dc";
  }

  static const char* value(const ::object_map_msgs::KeyframeTrans_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7867742dbcaa5ee3ULL;
  static const uint64_t static_value2 = 0x2577032d2a8da3dcULL;
};

template<class ContainerAllocator>
struct DataType< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> >
{
  static const char* value()
  {
    return "object_map_msgs/KeyframeTrans";
  }

  static const char* value(const ::object_map_msgs::KeyframeTrans_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"std_msgs/Float64MultiArray keyframeTrans\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Float64MultiArray\n"
"# Please look at the MultiArrayLayout message definition for\n"
"# documentation on all multiarrays.\n"
"\n"
"MultiArrayLayout  layout        # specification of data layout\n"
"float64[]         data          # array of data\n"
"\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/MultiArrayLayout\n"
"# The multiarray declares a generic multi-dimensional array of a\n"
"# particular data type.  Dimensions are ordered from outer most\n"
"# to inner most.\n"
"\n"
"MultiArrayDimension[] dim # Array of dimension properties\n"
"uint32 data_offset        # padding elements at front of data\n"
"\n"
"# Accessors should ALWAYS be written in terms of dimension stride\n"
"# and specified outer-most dimension first.\n"
"# \n"
"# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]\n"
"#\n"
"# A standard, 3-channel 640x480 image with interleaved color channels\n"
"# would be specified as:\n"
"#\n"
"# dim[0].label  = \"height\"\n"
"# dim[0].size   = 480\n"
"# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)\n"
"# dim[1].label  = \"width\"\n"
"# dim[1].size   = 640\n"
"# dim[1].stride = 3*640 = 1920\n"
"# dim[2].label  = \"channel\"\n"
"# dim[2].size   = 3\n"
"# dim[2].stride = 3\n"
"#\n"
"# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/MultiArrayDimension\n"
"string label   # label of given dimension\n"
"uint32 size    # size of given dimension (in type units)\n"
"uint32 stride  # stride of given dimension\n"
;
  }

  static const char* value(const ::object_map_msgs::KeyframeTrans_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.keyframeTrans);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct KeyframeTrans_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::object_map_msgs::KeyframeTrans_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::object_map_msgs::KeyframeTrans_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "keyframeTrans: ";
    s << std::endl;
    Printer< ::std_msgs::Float64MultiArray_<ContainerAllocator> >::stream(s, indent + "  ", v.keyframeTrans);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OBJECT_MAP_MSGS_MESSAGE_KEYFRAMETRANS_H
