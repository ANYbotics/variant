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
#include <variant_topic_tools/BuiltinPointer.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
BuiltinVariant::ValueImplT<T>::ValueImplT(const Pointer<T>& value) :
  value(value) {
}

template <typename T>
BuiltinVariant::ValueImplT<T>::ValueImplT(const ValueImplT<T>& src) :
  value(src.value) {
}

template <typename T>
BuiltinVariant::ValueImplT<T>::~ValueImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
void BuiltinVariant::ValueImplT<T>::set(const Pointer<T>& value) {
  this->value = value;
}

template <typename T>
void BuiltinVariant::ValueImplT<T>::setValue(const T& value) {
  if (!this->value)
    this->value = BuiltinPointer<T>(new T());
  
  *this->value = value;
}

template <typename T>
T& BuiltinVariant::ValueImplT<T>::getValue() {
  if (!this->value)
    this->value = BuiltinPointer<T>(new T());
  
  return *this->value;
}

template <typename T>
const T& BuiltinVariant::ValueImplT<T>::getValue() const {
  if (!this->value) {
    static T value = T();
    return value;
  }
  else
    return *this->value;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> BuiltinVariant BuiltinVariant::create(const
    DataType& type, const Pointer<T>& value) {
  BuiltinVariant variant;  
  
  variant.type = type;
  variant.value.reset(new ValueImplT<T>());
  
  return variant;
}

template <typename T>
Variant::ValuePtr BuiltinVariant::ValueImplT<T>::clone() const {
  return BuiltinVariant::ValuePtr(new ValueImplT<T>(*this));
}

}
