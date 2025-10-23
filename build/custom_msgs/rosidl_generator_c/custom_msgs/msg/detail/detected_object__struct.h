// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/DetectedObject.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'class_name'
// Member 'frame_id'
#include "rosidl_runtime_c/string.h"
// Member 'point'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/DetectedObject in the package custom_msgs.
/**
  * 탐지된 하나의 객체에 대한 정보
 */
typedef struct custom_msgs__msg__DetectedObject
{
  /// ROS Header (탐지 시점의 타임스탬프 및 프레임 ID)
  std_msgs__msg__Header header;
  /// 클래스 이름 (예: car, red, green, dummy 등)
  rosidl_runtime_c__String class_name;
  /// 이미지 상의 중심 좌표 (픽셀 단위)
  int32_t x_center;
  int32_t y_center;
  /// 깊이 카메라 기반 거리 정보 (단위: meter)
  float distance;
  /// 카메라 좌표계 기준 3D 좌표
  geometry_msgs__msg__Point point;
  /// 변환된 map 좌표계 기준 3D 좌표 (TF 변환 성공 시)
  float map_x;
  float map_y;
  float map_z;
  /// 변환된 좌표계의 frame_id 및 timestamp
  rosidl_runtime_c__String frame_id;
  builtin_interfaces__msg__Time stamp;
} custom_msgs__msg__DetectedObject;

// Struct for a sequence of custom_msgs__msg__DetectedObject.
typedef struct custom_msgs__msg__DetectedObject__Sequence
{
  custom_msgs__msg__DetectedObject * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__DetectedObject__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__DETECTED_OBJECT__STRUCT_H_
