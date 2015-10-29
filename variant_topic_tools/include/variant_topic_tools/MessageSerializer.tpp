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

#include <variant_topic_tools/Variant.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
MessageSerializer::ImplT<T>::ImplT() {
}

template <typename T>
MessageSerializer::ImplT<T>::~ImplT() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> MessageSerializer MessageSerializer::create() {
  MessageSerializer messageSerializer;
  messageSerializer.impl.reset(new ImplT<T>());
  
  return messageSerializer;
}

template <typename T>
void MessageSerializer::ImplT<T>::serialize(ros::serialization::OStream&
    stream, const Variant& value) {
  ros::serialization::serialize(stream, value.template getValue<T>());
}

template <typename T>
void MessageSerializer::ImplT<T>::deserialize(ros::serialization::IStream&
    stream, Variant& value) {
  ros::serialization::deserialize(stream, value.template getValue<T>());
}

template <typename T>
void MessageSerializer::ImplT<T>::advance(ros::serialization::IStream&
    stream, const Variant& value) {
  stream.advance(ros::serialization::serializationLength(value.template
    getValue<T>()));
}

}
