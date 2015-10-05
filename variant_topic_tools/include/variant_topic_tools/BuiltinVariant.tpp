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

#include <variant_topic_tools/BuiltinDataType.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
BuiltinVariant::ValueT<T>::ValueT(const T& value) :
  value(value) {
}

template <typename T>
BuiltinVariant::ValueT<T>::ValueT(const ValueT<T>& src) :
  value(src.getValue()) {
}

template <typename T>
BuiltinVariant::ValueT<T>::~ValueT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
void BuiltinVariant::ValueT<T>::setValue(const T& value) {
  this->value = value;
}

template <typename T>
T& BuiltinVariant::ValueT<T>::getValue() {
  return this->value;
}

template <typename T>
const T& BuiltinVariant::ValueT<T>::getValue() const {
  return this->value;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> BuiltinVariant BuiltinVariant::create(const
    DataType& type) {
  BuiltinVariant variant;
  
  variant.type = type;
  variant.value.reset(new ValueT<T>());
  
  return variant;
}

template <typename T>
Variant::ValuePtr BuiltinVariant::ValueT<T>::clone() const {
  return BuiltinVariant::ValuePtr(new ValueT<T>(*this));
}

}
