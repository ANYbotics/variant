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

#include "variant_topic_tools/ArraySerializer.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ArraySerializer::ArraySerializer() {
}

ArraySerializer::ArraySerializer(const Serializer& elementSerializer, size_t
    numElements) {
  impl.reset(new ImplV(elementSerializer, numElements));
}

ArraySerializer::ArraySerializer(const ArraySerializer& src) :
  Serializer(src) {
}

ArraySerializer::ArraySerializer(const Serializer& src) :
  Serializer(src) {
  if (impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(impl));
}

ArraySerializer::~ArraySerializer() {
}

ArraySerializer::Impl::Impl() {
}

ArraySerializer::Impl::~Impl() {
}

ArraySerializer::ImplV::ImplV(const Serializer& elementSerializer, size_t
    numElements) {
}

ArraySerializer::ImplV::~ImplV() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ArraySerializer::ImplV::serialize(ros::serialization::OStream& stream,
    const Variant& value) {
}

void ArraySerializer::ImplV::deserialize(ros::serialization::IStream& stream,
    Variant& value) {
}

void ArraySerializer::ImplV::advance(ros::serialization::IStream& stream) {
}

}
