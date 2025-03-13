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

/** \file MessageTraits.h
 * \brief Header file providing the variant message traits
 */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_TRAITS_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_TRAITS_H

#include <ros/ros.h>

#include <variant_topic_tools/Message.h>

namespace ros {
namespace message_traits {
template <>
struct IsMessage<variant_topic_tools::Message> : TrueType {};
template <>
struct IsMessage<const variant_topic_tools::Message> : TrueType {};

template <>
struct MD5Sum<variant_topic_tools::Message> {
  static const char* value(const variant_topic_tools::Message& message);
  static const char* value();
};

template <>
struct DataType<variant_topic_tools::Message> {
  static const char* value(const variant_topic_tools::Message& message);
  static const char* value();
};

template <>
struct Definition<variant_topic_tools::Message> {
  static const char* value(const variant_topic_tools::Message& message);
};
}  // namespace message_traits
}  // namespace ros

#endif
