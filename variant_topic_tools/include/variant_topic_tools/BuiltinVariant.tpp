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

#include <variant_topic_tools/BuiltinPointer.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
BuiltinVariant::ValueImplT<T>::ValueImplT(const Pointer<ValueType>& value) :
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
void BuiltinVariant::ValueImplT<T>::set(const Pointer<ValueType>& value) {
  this->value = value;
}

template <typename T>
void BuiltinVariant::ValueImplT<T>::setValue(const Variant::Value& value) {
  if (!this->value)
    this->value = BuiltinPointer<T>(new ValueType());
  
  *this->value = dynamic_cast<const ValueImplT<T>&>(value).getValue();
}

template <typename T>
typename BuiltinVariant::ValueImplT<T>::ValueType& BuiltinVariant::
    ValueImplT<T>::getValue() {
  if (!this->value)
    this->value = BuiltinPointer<T>(new ValueType());
  
  return *this->value;
}

template <typename T>
const typename BuiltinVariant::ValueImplT<T>::ValueType& BuiltinVariant::
    ValueImplT<T>::getValue() const {
  if (!this->value) {
    static ValueType value = ValueType();
    return value;
  }
  else
    return *this->value;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> BuiltinVariant BuiltinVariant::create(const DataType&
    type) {
  BuiltinVariant variant;  
  
  variant.type = type;
  variant.value.reset(new ValueImplT<T>());
  
  return variant;
}

template <typename T>
Variant::ValuePtr BuiltinVariant::ValueImplT<T>::clone() const {
  return BuiltinVariant::ValuePtr(new ValueImplT<T>(*this));
}

template <typename T>
void BuiltinVariant::ValueImplT<T>::read(std::istream& stream) {
  if (!this->value)
    this->value = BuiltinPointer<T>(new ValueType());
  
  BuiltinVariant::template read<T>(stream, *this->value);
}

template <typename T>
void BuiltinVariant::ValueImplT<T>::write(std::ostream& stream) const {
  if (!this->value) {
    static ValueType value = ValueType();
    BuiltinVariant::template write<T>(stream, value);
  }
  else
    BuiltinVariant::template write<T>(stream, *this->value);
}

template <typename T> void BuiltinVariant::read(std::istream& stream, typename
    type_traits::BuiltinType<T>::ValueType& value, typename boost::enable_if<
    boost::is_same<T, bool> >::type*) {
  typedef typename type_traits::BuiltinType<T>::ValueType ValueType;
  
  std::string stringValue;
  stream >> stringValue;
  
  value = (stringValue == "true") ? ValueType(1) : ValueType(0);
}

template <typename T> void BuiltinVariant::read(std::istream& stream, typename
    type_traits::BuiltinType<T>::ValueType& value, typename boost::disable_if<
    boost::is_same<T, bool> >::type*) {
  Variant::template read<T>(stream, value);
}

template <typename T> void BuiltinVariant::write(std::ostream& stream, const
    typename type_traits::BuiltinType<T>::ValueType& value, typename boost::
    enable_if<boost::is_same<T, bool> >::type*) {
  stream << (value ? "true" : "false");
}

template <typename T> void BuiltinVariant::write(std::ostream& stream, const
    typename type_traits::BuiltinType<T>::ValueType& value, typename boost::
    disable_if<boost::is_same<T, bool> >::type*) {
  Variant::template write<T>(stream, value);
}

}
