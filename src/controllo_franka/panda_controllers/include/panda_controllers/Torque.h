// Generated by gencpp from file panda_controllers/Torque.msg
// DO NOT EDIT!


#ifndef PANDA_CONTROLLERS_MESSAGE_TORQUE_H
#define PANDA_CONTROLLERS_MESSAGE_TORQUE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace panda_controllers
{
template <class ContainerAllocator>
struct Torque_
{
  typedef Torque_<ContainerAllocator> Type;

  Torque_()
    : tau()  {
    }
  Torque_(const ContainerAllocator& _alloc)
    : tau(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _tau_type;
  _tau_type tau;





  typedef boost::shared_ptr< ::panda_controllers::Torque_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::panda_controllers::Torque_<ContainerAllocator> const> ConstPtr;

}; // struct Torque_

typedef ::panda_controllers::Torque_<std::allocator<void> > Torque;

typedef boost::shared_ptr< ::panda_controllers::Torque > TorquePtr;
typedef boost::shared_ptr< ::panda_controllers::Torque const> TorqueConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::panda_controllers::Torque_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::panda_controllers::Torque_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::panda_controllers::Torque_<ContainerAllocator1> & lhs, const ::panda_controllers::Torque_<ContainerAllocator2> & rhs)
{
  return lhs.tau == rhs.tau;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::panda_controllers::Torque_<ContainerAllocator1> & lhs, const ::panda_controllers::Torque_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace panda_controllers

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::panda_controllers::Torque_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::panda_controllers::Torque_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::panda_controllers::Torque_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::panda_controllers::Torque_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::panda_controllers::Torque_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::panda_controllers::Torque_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::panda_controllers::Torque_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6488f72361a7c3710faf21208a8c781c";
  }

  static const char* value(const ::panda_controllers::Torque_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6488f72361a7c371ULL;
  static const uint64_t static_value2 = 0x0faf21208a8c781cULL;
};

template<class ContainerAllocator>
struct DataType< ::panda_controllers::Torque_<ContainerAllocator> >
{
  static const char* value()
  {
    return "panda_controllers/Torque";
  }

  static const char* value(const ::panda_controllers::Torque_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::panda_controllers::Torque_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[] tau\n"
;
  }

  static const char* value(const ::panda_controllers::Torque_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::panda_controllers::Torque_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.tau);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Torque_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::panda_controllers::Torque_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::panda_controllers::Torque_<ContainerAllocator>& v)
  {
    s << indent << "tau[]" << std::endl;
    for (size_t i = 0; i < v.tau.size(); ++i)
    {
      s << indent << "  tau[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.tau[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PANDA_CONTROLLERS_MESSAGE_TORQUE_H
