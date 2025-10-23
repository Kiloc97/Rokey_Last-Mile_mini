// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/DetectedObject.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/detected_object__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectedObject_stamp
{
public:
  explicit Init_DetectedObject_stamp(::custom_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::DetectedObject stamp(::custom_msgs::msg::DetectedObject::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_frame_id
{
public:
  explicit Init_DetectedObject_frame_id(::custom_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_stamp frame_id(::custom_msgs::msg::DetectedObject::_frame_id_type arg)
  {
    msg_.frame_id = std::move(arg);
    return Init_DetectedObject_stamp(msg_);
  }

private:
  ::custom_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_map_z
{
public:
  explicit Init_DetectedObject_map_z(::custom_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_frame_id map_z(::custom_msgs::msg::DetectedObject::_map_z_type arg)
  {
    msg_.map_z = std::move(arg);
    return Init_DetectedObject_frame_id(msg_);
  }

private:
  ::custom_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_map_y
{
public:
  explicit Init_DetectedObject_map_y(::custom_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_map_z map_y(::custom_msgs::msg::DetectedObject::_map_y_type arg)
  {
    msg_.map_y = std::move(arg);
    return Init_DetectedObject_map_z(msg_);
  }

private:
  ::custom_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_map_x
{
public:
  explicit Init_DetectedObject_map_x(::custom_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_map_y map_x(::custom_msgs::msg::DetectedObject::_map_x_type arg)
  {
    msg_.map_x = std::move(arg);
    return Init_DetectedObject_map_y(msg_);
  }

private:
  ::custom_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_point
{
public:
  explicit Init_DetectedObject_point(::custom_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_map_x point(::custom_msgs::msg::DetectedObject::_point_type arg)
  {
    msg_.point = std::move(arg);
    return Init_DetectedObject_map_x(msg_);
  }

private:
  ::custom_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_distance
{
public:
  explicit Init_DetectedObject_distance(::custom_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_point distance(::custom_msgs::msg::DetectedObject::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_DetectedObject_point(msg_);
  }

private:
  ::custom_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_y_center
{
public:
  explicit Init_DetectedObject_y_center(::custom_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_distance y_center(::custom_msgs::msg::DetectedObject::_y_center_type arg)
  {
    msg_.y_center = std::move(arg);
    return Init_DetectedObject_distance(msg_);
  }

private:
  ::custom_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_x_center
{
public:
  explicit Init_DetectedObject_x_center(::custom_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_y_center x_center(::custom_msgs::msg::DetectedObject::_x_center_type arg)
  {
    msg_.x_center = std::move(arg);
    return Init_DetectedObject_y_center(msg_);
  }

private:
  ::custom_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_class_name
{
public:
  explicit Init_DetectedObject_class_name(::custom_msgs::msg::DetectedObject & msg)
  : msg_(msg)
  {}
  Init_DetectedObject_x_center class_name(::custom_msgs::msg::DetectedObject::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_DetectedObject_x_center(msg_);
  }

private:
  ::custom_msgs::msg::DetectedObject msg_;
};

class Init_DetectedObject_header
{
public:
  Init_DetectedObject_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectedObject_class_name header(::custom_msgs::msg::DetectedObject::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DetectedObject_class_name(msg_);
  }

private:
  ::custom_msgs::msg::DetectedObject msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::DetectedObject>()
{
  return custom_msgs::msg::builder::Init_DetectedObject_header();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__DETECTED_OBJECT__BUILDER_HPP_
