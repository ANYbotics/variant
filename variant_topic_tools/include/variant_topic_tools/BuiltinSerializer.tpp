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
BuiltinSerializer::ImplT<T>::ImplT() {
}

template <typename T>
BuiltinSerializer::ImplT<T>::~ImplT() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> BuiltinSerializer BuiltinSerializer::create() {
  BuiltinSerializer builtinSerializer;
  builtinSerializer.impl.reset(new ImplT<T>());
  
  return builtinSerializer;
}

template <typename T>
void BuiltinSerializer::ImplT<T>::serialize(ros::serialization::OStream&
    stream, const Variant& value) {
  ros::serialization::serialize(stream, value.template getValue<T>());
}

template <typename T>
void BuiltinSerializer::ImplT<T>::deserialize(ros::serialization::IStream&
    stream, Variant& value) {
  ros::serialization::deserialize(stream, value.template getValue<T>());
}

template <typename T>
void BuiltinSerializer::ImplT<T>::advance(ros::serialization::IStream&
    stream) {
  Serializer::template advance<T>(stream);
}

}
