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

/** \file Serialization.h
 * \brief Header file providing the variant message serialization
 */

#ifndef TOPIC_VARIANT_SERIALIZATION_H
#define TOPIC_VARIANT_SERIALIZATION_H

#include <ros/ros.h>

#include <variant_topic_tools/Message.h>

namespace ros {
namespace serialization {
template <>
struct Serializer<variant_topic_tools::Message> {
  template <class Stream>
  inline static void read(Stream& stream, variant_topic_tools::Message& message);
  template <class Stream>
  inline static void write(Stream& stream, const variant_topic_tools::Message& message);
  inline static uint32_t serializedLength(const variant_topic_tools::Message& message);
};

template <>
struct PreDeserialize<variant_topic_tools::Message> {
  static void notify(const PreDeserializeParams<variant_topic_tools::Message>& params);
};
}  // namespace serialization
}  // namespace ros

#include <variant_topic_tools/Serialization.tpp>

#endif
