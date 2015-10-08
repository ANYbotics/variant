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

#include <variant_topic_tools/ArrayTypeTraits.h>
#include <variant_topic_tools/Variant.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T, size_t N>
ArraySerializer::ImplT<T, N>::ImplT() {
}

template <typename T, size_t N>
ArraySerializer::ImplT<T, N>::~ImplT() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T, size_t N> ArraySerializer ArraySerializer::create() {
  ArraySerializer arraySerializer;
  arraySerializer.impl.reset(new ImplT<T, N>());
  
  return arraySerializer;
}

template <typename T, size_t N>
void ArraySerializer::ImplT<T, N>::serialize(ros::serialization::OStream&
    stream, const Variant& value) {
  ros::serialization::serialize(stream, value.template getValue<typename
    ArrayTypeTraits::ToArray<T, N>::ArrayType>());
}

template <typename T, size_t N>
void ArraySerializer::ImplT<T, N>::deserialize(ros::serialization::IStream&
    stream, Variant& value) {
  ros::serialization::deserialize(stream, value.template getValue<typename
    ArrayTypeTraits::ToArray<T, N>::ArrayType>());
}

template <typename T, size_t N>
void ArraySerializer::ImplT<T, N>::advance(ros::serialization::IStream&
    stream) {
  Serializer::template advance<typename ArrayTypeTraits::ToArray<T, N>::
    ArrayType>(stream);
}

}
