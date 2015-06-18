/* Auto-generated by genmsg_cpp for file /tmp/buildd/ros-groovy-web-interface-0.5.0/debian/ros-groovy-web-interface/opt/ros/groovy/stacks/web_interface/web_msgs/msg/WebEvent.msg */
#ifndef WEB_MSGS_MESSAGE_WEBEVENT_H
#define WEB_MSGS_MESSAGE_WEBEVENT_H
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


namespace web_msgs
{
template <class ContainerAllocator>
struct WebEvent_ {
  typedef WebEvent_<ContainerAllocator> Type;

  WebEvent_()
  : source()
  , type()
  , data()
  {
  }

  WebEvent_(const ContainerAllocator& _alloc)
  : source(_alloc)
  , type(_alloc)
  , data(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _source_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  source;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  type;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _data_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  data;


  typedef boost::shared_ptr< ::web_msgs::WebEvent_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::web_msgs::WebEvent_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct WebEvent
typedef  ::web_msgs::WebEvent_<std::allocator<void> > WebEvent;

typedef boost::shared_ptr< ::web_msgs::WebEvent> WebEventPtr;
typedef boost::shared_ptr< ::web_msgs::WebEvent const> WebEventConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::web_msgs::WebEvent_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::web_msgs::WebEvent_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace web_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::web_msgs::WebEvent_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::web_msgs::WebEvent_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::web_msgs::WebEvent_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4f05e2e1608bf7d788fa5e654f805aeb";
  }

  static const char* value(const  ::web_msgs::WebEvent_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4f05e2e1608bf7d7ULL;
  static const uint64_t static_value2 = 0x88fa5e654f805aebULL;
};

template<class ContainerAllocator>
struct DataType< ::web_msgs::WebEvent_<ContainerAllocator> > {
  static const char* value() 
  {
    return "web_msgs/WebEvent";
  }

  static const char* value(const  ::web_msgs::WebEvent_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::web_msgs::WebEvent_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string source		# who or what triggered the event\n\
string type		# type of event (e.g. login, logout)\n\
string data		# any data about the event\n\
";
  }

  static const char* value(const  ::web_msgs::WebEvent_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::web_msgs::WebEvent_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.source);
    stream.next(m.type);
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct WebEvent_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::web_msgs::WebEvent_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::web_msgs::WebEvent_<ContainerAllocator> & v) 
  {
    s << indent << "source: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.source);
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "data: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.data);
  }
};


} // namespace message_operations
} // namespace ros

#endif // WEB_MSGS_MESSAGE_WEBEVENT_H
