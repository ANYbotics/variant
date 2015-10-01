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

#include <typeinfo>

#include <variant_topic_tools/Exceptions.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T> Variant::Variant(const T& src) {
  TypeTraits::assign(*this, src);
}

template <typename T>
Variant::ValueT<T>::ValueT() {
}

template <typename T>
Variant::ValueT<T>::~ValueT() {
}

template <typename T>
Variant::ValueImplT<T>::ValueImplT(const T& value) :
  value(value) {
}

template <typename T>
Variant::ValueImplT<T>::ValueImplT(const ValueT<T>& src) :
  value(src.getValue()) {
}

template <typename T>
Variant::ValueImplT<T>::~ValueImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T> void Variant::setValue(const T& value) {
  if (!this->type.isValid()) {
    this->type = DataType(typeid(T));
    
    if (this->type.isValid())
      this->value.reset(new ValueImplT<T>(value));
    else
      throw InvalidDataTypeException();
  }
  else if (typeid(T) == this->type.getTypeInfo()) {
    if (this->value)
      boost::dynamic_pointer_cast<ValueT<T> >(this->value)->setValue(value);
    else
      this->value.reset(new ValueImplT<T>(value));
  }
  else
    throw DataTypeMismatchException(type.getIdentifier(),
      DataType(typeid(T)).getIdentifier());
}

template <typename T> T& Variant::getValue() {
  if (!this->type.isValid()) {
    this->type = DataType(typeid(T));
    
    if (this->type.isValid()) {
      this->value.reset(new ValueImplT<T>());
      
      return boost::dynamic_pointer_cast<ValueT<T> >(this->value)->getValue();
    }
    else
      throw InvalidDataTypeException();
  }
  else if (typeid(T) == this->type.getTypeInfo()) {
    if (!this->value)
      this->value.reset(new ValueImplT<T>());
    
    return boost::dynamic_pointer_cast<ValueT<T> >(this->value)->getValue();
  }
  else
    throw DataTypeMismatchException(type.getIdentifier(),
      DataType(typeid(T)).getIdentifier());
}

template <typename T> const T& Variant::getValue() const {
  if (this->type.isValid()) {
    if (typeid(T) == this->type.getTypeInfo()) {
      if (!this->value) {
        static T value = T();
        return value;
      }
      else 
        return boost::dynamic_pointer_cast<ValueT<T> >(this->value)->
          getValue();
    }
    else
      throw DataTypeMismatchException(type.getIdentifier(),
        DataType(typeid(T)).getIdentifier());
  }
  else
    throw InvalidDataTypeException();
}

template <typename T>
bool Variant::ValueT<T>::isEqual(const Value& value) const {
  return TypeTraits::isEqual(dynamic_cast<const ValueT<T>&>(value).getValue(),
    this->getValue());
}

template <typename T>
void Variant::ValueImplT<T>::setValue(const T& value) {
  this->value = value;
}

template <typename T>
T& Variant::ValueImplT<T>::getValue() {
  return this->value;
}

template <typename T>
const T& Variant::ValueImplT<T>::getValue() const {
  return this->value;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> void Variant::TypeTraits::assign(Variant& dst, const
    T& src, typename boost::enable_if<boost::is_base_of<Variant, T> >::type*
    dummy) {
  dst.type = src.type;
  dst.value = src.value ? src.value->clone() : ValuePtr();
}

template <typename T> void Variant::TypeTraits::assign(Variant& dst, const
    T& src, typename boost::disable_if<boost::is_base_of<Variant, T> >::type*
    dummy) {
  dst.template setValue<T>(src);
}

template <typename T> bool Variant::TypeTraits::isEqual(const T& lhs, const
    T& rhs, typename boost::enable_if<boost::has_equal_to<T, T, bool> >::type*
    dummy) {
  return (lhs == rhs);
}

template <typename T, size_t N> bool Variant::TypeTraits::isEqual(const
    boost::array<T, N>& lhs, const boost::array<T, N>& rhs, typename
    boost::enable_if<boost::has_equal_to<T, T, bool> >::type* dummy) {
  return (lhs == rhs);
}

template <typename T> bool Variant::TypeTraits::isEqual(const std::vector<T>&
    lhs, const std::vector<T>& rhs, typename boost::enable_if<boost::
    has_equal_to<T, T, bool> >::type* dummy) {
  return (lhs == rhs);
}

template <typename T> bool Variant::TypeTraits::isEqual(const T& lhs, const
    T& rhs, typename boost::disable_if<boost::has_equal_to<T, T, bool> >::type*
    dummy) {
  throw InvalidOperationException(
    "Comparing two variants of non-comparable type");
}

template <typename T, size_t N> bool Variant::TypeTraits::isEqual(const
    boost::array<T, N>& lhs, const boost::array<T, N>& rhs, typename
    boost::disable_if<boost::has_equal_to<T, T, bool> >::type* dummy) {
  throw InvalidOperationException(
    "Comparing two variants of non-comparable type");
}

template <typename T> bool Variant::TypeTraits::isEqual(const std::vector<T>&
    lhs, const std::vector<T>& rhs, typename boost::disable_if<boost::
    has_equal_to<T, T, bool> >::type* dummy) {
  throw InvalidOperationException(
    "Comparing two variants of non-comparable type");
}

template <typename T> void Variant::TypeTraits::read(std::istream& stream,
    T& value, typename boost::enable_if<boost::has_right_shift<std::istream,
    T&> >::type* dummy) {
  stream >> value;
}

template <typename T> void Variant::TypeTraits::read(std::istream& stream,
    T& value, typename boost::disable_if<boost::has_right_shift<std::istream,
    T&> >::type* dummy) {
  throw InvalidOperationException(
    "Reading a variant of non-readable type");
}

template <typename T> void Variant::TypeTraits::write(std::ostream& stream,
    const T& value, typename boost::enable_if<boost::has_left_shift<std::
    ostream, const T&> >::type* dummy) {
  stream << value;
}

template <typename T> void Variant::TypeTraits::write(std::ostream& stream,
    const T& value, typename boost::disable_if<boost::has_left_shift<std::
    ostream, const T&> >::type* dummy) {
}

template <typename T>
void Variant::ValueT<T>::read(std::istream& stream) {
  TypeTraits::read(stream, this->getValue());
}

template <typename T>
void Variant::ValueT<T>::write(std::ostream& stream) const {
  TypeTraits::write(stream, this->getValue());
}

template <typename T>
Variant::ValuePtr Variant::ValueImplT<T>::clone() const {
  return Variant::ValuePtr(new ValueImplT<T>(*this));
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

template <typename T> Variant& Variant::operator=(const T& src) {
  TypeTraits::assign(*this, src);
  return *this;
}

template <typename T> Variant::operator const T&() const {
  return this->template getValue<T>();
}

}
