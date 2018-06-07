// Generated by gencpp from file ardrone_autonomy/navdata_kalman_pressure.msg
// DO NOT EDIT!


#ifndef ARDRONE_AUTONOMY_MESSAGE_NAVDATA_KALMAN_PRESSURE_H
#define ARDRONE_AUTONOMY_MESSAGE_NAVDATA_KALMAN_PRESSURE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace ardrone_autonomy
{
template <class ContainerAllocator>
struct navdata_kalman_pressure_
{
  typedef navdata_kalman_pressure_<ContainerAllocator> Type;

  navdata_kalman_pressure_()
    : header()
    , drone_time(0.0)
    , tag(0)
    , size(0)
    , offset_pressure(0.0)
    , est_z(0.0)
    , est_zdot(0.0)
    , est_bias_PWM(0.0)
    , est_biais_pression(0.0)
    , offset_US(0.0)
    , prediction_US(0.0)
    , cov_alt(0.0)
    , cov_PWM(0.0)
    , cov_vitesse(0.0)
    , bool_effet_sol(0)
    , somme_inno(0.0)
    , flag_rejet_US(0)
    , u_multisinus(0.0)
    , gaz_altitude(0.0)
    , Flag_multisinus(0)
    , Flag_multisinus_debut(0)  {
    }
  navdata_kalman_pressure_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , drone_time(0.0)
    , tag(0)
    , size(0)
    , offset_pressure(0.0)
    , est_z(0.0)
    , est_zdot(0.0)
    , est_bias_PWM(0.0)
    , est_biais_pression(0.0)
    , offset_US(0.0)
    , prediction_US(0.0)
    , cov_alt(0.0)
    , cov_PWM(0.0)
    , cov_vitesse(0.0)
    , bool_effet_sol(0)
    , somme_inno(0.0)
    , flag_rejet_US(0)
    , u_multisinus(0.0)
    , gaz_altitude(0.0)
    , Flag_multisinus(0)
    , Flag_multisinus_debut(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _drone_time_type;
  _drone_time_type drone_time;

   typedef uint16_t _tag_type;
  _tag_type tag;

   typedef uint16_t _size_type;
  _size_type size;

   typedef float _offset_pressure_type;
  _offset_pressure_type offset_pressure;

   typedef float _est_z_type;
  _est_z_type est_z;

   typedef float _est_zdot_type;
  _est_zdot_type est_zdot;

   typedef float _est_bias_PWM_type;
  _est_bias_PWM_type est_bias_PWM;

   typedef float _est_biais_pression_type;
  _est_biais_pression_type est_biais_pression;

   typedef float _offset_US_type;
  _offset_US_type offset_US;

   typedef float _prediction_US_type;
  _prediction_US_type prediction_US;

   typedef float _cov_alt_type;
  _cov_alt_type cov_alt;

   typedef float _cov_PWM_type;
  _cov_PWM_type cov_PWM;

   typedef float _cov_vitesse_type;
  _cov_vitesse_type cov_vitesse;

   typedef int32_t _bool_effet_sol_type;
  _bool_effet_sol_type bool_effet_sol;

   typedef float _somme_inno_type;
  _somme_inno_type somme_inno;

   typedef int32_t _flag_rejet_US_type;
  _flag_rejet_US_type flag_rejet_US;

   typedef float _u_multisinus_type;
  _u_multisinus_type u_multisinus;

   typedef float _gaz_altitude_type;
  _gaz_altitude_type gaz_altitude;

   typedef int32_t _Flag_multisinus_type;
  _Flag_multisinus_type Flag_multisinus;

   typedef int32_t _Flag_multisinus_debut_type;
  _Flag_multisinus_debut_type Flag_multisinus_debut;




  typedef boost::shared_ptr< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> const> ConstPtr;

}; // struct navdata_kalman_pressure_

typedef ::ardrone_autonomy::navdata_kalman_pressure_<std::allocator<void> > navdata_kalman_pressure;

typedef boost::shared_ptr< ::ardrone_autonomy::navdata_kalman_pressure > navdata_kalman_pressurePtr;
typedef boost::shared_ptr< ::ardrone_autonomy::navdata_kalman_pressure const> navdata_kalman_pressureConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ardrone_autonomy

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'ardrone_autonomy': ['/home/cdio/cdio_ws/src/ardrone_autonomy/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> >
{
  static const char* value()
  {
    return "70734b6caff0fb7ea6fc88ffeea5cde5";
  }

  static const char* value(const ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x70734b6caff0fb7eULL;
  static const uint64_t static_value2 = 0xa6fc88ffeea5cde5ULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ardrone_autonomy/navdata_kalman_pressure";
  }

  static const char* value(const ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float64 drone_time\n\
uint16 tag\n\
uint16 size\n\
float32 offset_pressure\n\
float32 est_z\n\
float32 est_zdot\n\
float32 est_bias_PWM\n\
float32 est_biais_pression\n\
float32 offset_US\n\
float32 prediction_US\n\
float32 cov_alt\n\
float32 cov_PWM\n\
float32 cov_vitesse\n\
int32 bool_effet_sol\n\
float32 somme_inno\n\
int32 flag_rejet_US\n\
float32 u_multisinus\n\
float32 gaz_altitude\n\
int32 Flag_multisinus\n\
int32 Flag_multisinus_debut\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.drone_time);
      stream.next(m.tag);
      stream.next(m.size);
      stream.next(m.offset_pressure);
      stream.next(m.est_z);
      stream.next(m.est_zdot);
      stream.next(m.est_bias_PWM);
      stream.next(m.est_biais_pression);
      stream.next(m.offset_US);
      stream.next(m.prediction_US);
      stream.next(m.cov_alt);
      stream.next(m.cov_PWM);
      stream.next(m.cov_vitesse);
      stream.next(m.bool_effet_sol);
      stream.next(m.somme_inno);
      stream.next(m.flag_rejet_US);
      stream.next(m.u_multisinus);
      stream.next(m.gaz_altitude);
      stream.next(m.Flag_multisinus);
      stream.next(m.Flag_multisinus_debut);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct navdata_kalman_pressure_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ardrone_autonomy::navdata_kalman_pressure_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "drone_time: ";
    Printer<double>::stream(s, indent + "  ", v.drone_time);
    s << indent << "tag: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.tag);
    s << indent << "size: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.size);
    s << indent << "offset_pressure: ";
    Printer<float>::stream(s, indent + "  ", v.offset_pressure);
    s << indent << "est_z: ";
    Printer<float>::stream(s, indent + "  ", v.est_z);
    s << indent << "est_zdot: ";
    Printer<float>::stream(s, indent + "  ", v.est_zdot);
    s << indent << "est_bias_PWM: ";
    Printer<float>::stream(s, indent + "  ", v.est_bias_PWM);
    s << indent << "est_biais_pression: ";
    Printer<float>::stream(s, indent + "  ", v.est_biais_pression);
    s << indent << "offset_US: ";
    Printer<float>::stream(s, indent + "  ", v.offset_US);
    s << indent << "prediction_US: ";
    Printer<float>::stream(s, indent + "  ", v.prediction_US);
    s << indent << "cov_alt: ";
    Printer<float>::stream(s, indent + "  ", v.cov_alt);
    s << indent << "cov_PWM: ";
    Printer<float>::stream(s, indent + "  ", v.cov_PWM);
    s << indent << "cov_vitesse: ";
    Printer<float>::stream(s, indent + "  ", v.cov_vitesse);
    s << indent << "bool_effet_sol: ";
    Printer<int32_t>::stream(s, indent + "  ", v.bool_effet_sol);
    s << indent << "somme_inno: ";
    Printer<float>::stream(s, indent + "  ", v.somme_inno);
    s << indent << "flag_rejet_US: ";
    Printer<int32_t>::stream(s, indent + "  ", v.flag_rejet_US);
    s << indent << "u_multisinus: ";
    Printer<float>::stream(s, indent + "  ", v.u_multisinus);
    s << indent << "gaz_altitude: ";
    Printer<float>::stream(s, indent + "  ", v.gaz_altitude);
    s << indent << "Flag_multisinus: ";
    Printer<int32_t>::stream(s, indent + "  ", v.Flag_multisinus);
    s << indent << "Flag_multisinus_debut: ";
    Printer<int32_t>::stream(s, indent + "  ", v.Flag_multisinus_debut);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARDRONE_AUTONOMY_MESSAGE_NAVDATA_KALMAN_PRESSURE_H
