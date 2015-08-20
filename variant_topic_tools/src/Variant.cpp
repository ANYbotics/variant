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

#include "variant_topic_tools/Variant.h"
#include "variant_topic_tools/VariantArray.h"
#include "variant_topic_tools/VariantCollection.h"
#include "variant_topic_tools/VariantMessage.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Variant::Variant() {
}

Variant::Variant(const DataType& type) :
  type(type) {
}

Variant::Variant(const Variant& src) :
  type(src.type),
  value(src.value ? src.value->clone() : ValuePtr()) {
}

Variant::~Variant() {
}

Variant::Value::Value() {
}

Variant::Value::~Value() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const DataType& Variant::getType() const {
  return type;
}

bool Variant::hasType() const {
  return type.isValid();
}

bool Variant::isArray() const {
  if (value)
    return boost::dynamic_pointer_cast<VariantArray::Value>(value);
  else
    return false;
}

bool Variant::isCollection() const {
  if (value)
    return boost::dynamic_pointer_cast<VariantCollection::Value>(value);
  else
    return false;
}

bool Variant::isMessage() const {
  if (value)
    return boost::dynamic_pointer_cast<VariantMessage::Value>(value);
  else
    return false;
}

bool Variant::isEmpty() const {
  return !value;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

VariantArray Variant::asArray() const {
  VariantArray array;
  array.value = boost::dynamic_pointer_cast<VariantArray::Value>(value);

  return array;
}

VariantCollection Variant::asCollection() const {
  VariantCollection collection;
  collection.value = boost::dynamic_pointer_cast<VariantCollection::Value>(
    value);

  return collection;
}

VariantMessage Variant::asMessage() const {
  VariantMessage message;
  message.value = boost::dynamic_pointer_cast<VariantMessage::Value>(value);

  return message;
}

void Variant::clear() {
  type.clear();
  value.reset();
}

void Variant::read(std::istream& stream) {
  if (value)
    value->read(stream);
}

void Variant::write(std::ostream& stream) const {
  if (value)
    value->write(stream);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

Variant& Variant::operator=(const Variant& src) {
  type = src.type;
  value = src.value ? src.value->clone() : ValuePtr();
  
  return *this;
}

bool Variant::operator==(const Variant& variant) const {
  if ((type == variant.type) && value && variant.value)
    return value->isEqual(*variant.value);
  else
    return false;
}
    
bool Variant::operator!=(const Variant& variant) const {
  if ((type == variant.type) && value && variant.value)
    return !value->isEqual(*variant.value);
  else
    return true;
}

std::istream& operator>>(std::istream& stream, Variant& variant) {
  variant.read(stream);
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const Variant& variant) {
  variant.write(stream);
  return stream;
}

}
