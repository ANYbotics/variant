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

#include "variant_topic_tools/ArrayVariant.h"
#include "variant_topic_tools/BuiltinVariant.h"
#include "variant_topic_tools/CollectionVariant.h"
#include "variant_topic_tools/MessageVariant.h"
#include "variant_topic_tools/SharedVariant.h"
#include "variant_topic_tools/Variant.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Variant::Variant() {
}

Variant::Variant(const DataType& type) :
  type(type) {
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
    return boost::dynamic_pointer_cast<ArrayVariant::Value>(value);
  else
    return false;
}

bool Variant::isBuiltin() const {
  if (value)
    return boost::dynamic_pointer_cast<BuiltinVariant::Value>(value);
  else
    return false;
}

bool Variant::isCollection() const {
  if (value)
    return boost::dynamic_pointer_cast<CollectionVariant::Value>(value);
  else
    return false;
}

bool Variant::isMessage() const {
  if (value)
    return boost::dynamic_pointer_cast<MessageVariant::Value>(value);
  else
    return false;
}

bool Variant::isEmpty() const {
  return !value;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

ArrayVariant Variant::asArray() const {
  return SharedVariant(*this);
}

BuiltinVariant Variant::asBuiltin() const {
  return SharedVariant(*this);
}

CollectionVariant Variant::asCollection() const {
  return SharedVariant(*this);
}

MessageVariant Variant::asMessage() const {
  return SharedVariant(*this);
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
