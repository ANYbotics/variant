/******************************************************************************
 * Copyright (C) 2014 by Ralf Kaestner                                        *
 * ralf.kaestner@gmail.com                                                    *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file BuiltinTypeTraits.h
 * \brief Header file providing the built-in type traits
 */

#ifndef VARIANT_TOPIC_TOOLS_BUILTIN_TYPE_TRAITS_H
#define VARIANT_TOPIC_TOOLS_BUILTIN_TYPE_TRAITS_H

#include <string>

#include <boost/type_traits.hpp>
#include <boost/type_traits/ice.hpp>

#include <ros/duration.h>
#include <ros/message_traits.h>
#include <ros/time.h>

namespace variant_topic_tools {
namespace type_traits {
template <typename T, typename D = void>
struct IsBuiltin : public boost::type_traits::ice_or<boost::is_integral<T>::value, boost::is_floating_point<T>::value> {};

template <typename D>
struct IsBuiltin<std::string, D> : public boost::true_type {};

template <typename D>
struct IsBuiltin<ros::Duration, D> : public boost::true_type {};

template <typename D>
struct IsBuiltin<ros::Time, D> : public boost::true_type {};

template <typename T, typename D = void>
struct BuiltinType {
  using ValueType = T;
  using StreamType = T;
  using IsNumeric = boost::type_traits::ice_or<boost::is_integral<T>::value, boost::is_floating_point<T>::value>;
  using IsFixedSize = ros::message_traits::IsFixedSize<ValueType>;
  using IsSimple = ros::message_traits::IsSimple<ValueType>;
};

template <typename D>
struct BuiltinType<uint8_t, D> {
  using ValueType = uint8_t;
  using StreamType = uint32_t;
  using IsNumeric = boost::true_type;
  using IsFixedSize = ros::message_traits::IsFixedSize<ValueType>;
  using IsSimple = ros::message_traits::IsSimple<ValueType>;
};

template <typename D>
struct BuiltinType<int8_t, D> {
  using ValueType = int8_t;
  using StreamType = int32_t;
  using IsNumeric = boost::true_type;
  using IsFixedSize = ros::message_traits::IsFixedSize<ValueType>;
  using IsSimple = ros::message_traits::IsSimple<ValueType>;
};

template <typename D>
struct BuiltinType<bool, D> {
  using ValueType = uint8_t;
  using StreamType = std::string;
  using IsNumeric = boost::true_type;
  using IsFixedSize = ros::message_traits::IsFixedSize<ValueType>;
  using IsSimple = ros::message_traits::IsSimple<ValueType>;
};

template <typename D>
struct BuiltinType<ros::Duration, D> {
  using ValueType = ros::Duration;
  using StreamType = ros::Duration;
  using IsNumeric = boost::true_type;
  using IsFixedSize = ros::message_traits::IsFixedSize<ValueType>;
  using IsSimple = ros::message_traits::IsSimple<ValueType>;
};

template <typename D>
struct BuiltinType<ros::Time, D> {
  using ValueType = ros::Time;
  using StreamType = ros::Time;
  using IsNumeric = boost::true_type;
  using IsFixedSize = ros::message_traits::IsFixedSize<ValueType>;
  using IsSimple = ros::message_traits::IsSimple<ValueType>;
};

template <typename T>
struct ToBuiltinType {
  using BuiltinType = T;
};

template <size_t N>
struct ToBuiltinType<char[N]> {
  using BuiltinType = std::string;
};
}  // namespace type_traits
}  // namespace variant_topic_tools

#endif
