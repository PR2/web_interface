/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-web-interface-0.5.0/debian/ros-groovy-web-interface/opt/ros/groovy/stacks/web_interface/launchman/msg/AppUpdate.msg */
#ifndef LAUNCHMAN_MESSAGE_APPUPDATE_H
#define LAUNCHMAN_MESSAGE_APPUPDATE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace launchman
{
template <class ContainerAllocator>
struct AppUpdate_ {
  typedef AppUpdate_<ContainerAllocator> Type;

  AppUpdate_()
  : header()
  , taskid()
  , username()
  , status()
  , started()
  {
  }

  AppUpdate_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , taskid(_alloc)
  , username(_alloc)
  , status(_alloc)
  , started()
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _taskid_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  taskid;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _username_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  username;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  status;

  typedef ros::Time _started_type;
  ros::Time started;


  typedef boost::shared_ptr< ::launchman::AppUpdate_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::launchman::AppUpdate_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct AppUpdate
typedef  ::launchman::AppUpdate_<std::allocator<void> > AppUpdate;

typedef boost::shared_ptr< ::launchman::AppUpdate> AppUpdatePtr;
typedef boost::shared_ptr< ::launchman::AppUpdate const> AppUpdateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::launchman::AppUpdate_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::launchman::AppUpdate_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace launchman

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::launchman::AppUpdate_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::launchman::AppUpdate_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::launchman::AppUpdate_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5798525d2dcbad786f5d5d2c3dfd0cae";
  }

  static const char* value(const  ::launchman::AppUpdate_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5798525d2dcbad78ULL;
  static const uint64_t static_value2 = 0x6f5d5d2c3dfd0caeULL;
};

template<class ContainerAllocator>
struct DataType< ::launchman::AppUpdate_<ContainerAllocator> > {
  static const char* value() 
  {
    return "launchman/AppUpdate";
  }

  static const char* value(const  ::launchman::AppUpdate_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::launchman::AppUpdate_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
string taskid\n\
string username\n\
string status\n\
time started\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::launchman::AppUpdate_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::launchman::AppUpdate_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::launchman::AppUpdate_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::launchman::AppUpdate_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.taskid);
    stream.next(m.username);
    stream.next(m.status);
    stream.next(m.started);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct AppUpdate_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::launchman::AppUpdate_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::launchman::AppUpdate_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "taskid: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.taskid);
    s << indent << "username: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.username);
    s << indent << "status: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.status);
    s << indent << "started: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.started);
  }
};


} // namespace message_operations
} // namespace ros

#endif // LAUNCHMAN_MESSAGE_APPUPDATE_H

