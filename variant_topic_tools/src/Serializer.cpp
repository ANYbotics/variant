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

#include "variant_topic_tools/DataType.h"
#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/Serializer.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Serializer::Serializer() {
}

Serializer::Serializer(const DataType& dataType) {
  if (dataType.isValid())
    impl = dataType.createSerializer().impl;
}

Serializer::Serializer(const Serializer& src) :
  impl(src.impl) {
}

Serializer::~Serializer() {
}

Serializer::Impl::Impl() {
}

Serializer::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

bool Serializer::isValid() const {
  return impl;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Serializer::clear() {
  impl.reset();
}

void Serializer::serialize(ros::serialization::OStream& stream, const Variant&
    value) {
  if (impl)
    impl->serialize(stream, value);
  else
    throw InvalidSerializerException();
}

void Serializer::deserialize(ros::serialization::IStream& stream, Variant&
    value) {
  if (impl)
    impl->deserialize(stream, value);
  else
    throw InvalidSerializerException();
}

void Serializer::advance(ros::serialization::IStream& stream, const Variant&
    value) {
  if (impl)
    impl->advance(stream, value);
  else
    throw InvalidSerializerException();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

std::ostream& operator<<(std::ostream& stream, const Serializer& serializer) {
  return stream;
}

}
