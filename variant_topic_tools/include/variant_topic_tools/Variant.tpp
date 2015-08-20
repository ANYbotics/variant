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

template <typename T> Variant::Variant(const T& value) {
  this->template setValue<T>(value);
}

template <typename T>
Variant::ValueT<T>::ValueT(const T& value) :
  value(value) {
}

template <typename T>
Variant::ValueT<T>::ValueT(const ValueT<T>& src) :
  value(src.value) {
}

template <typename T>
Variant::ValueT<T>::~ValueT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T> void Variant::setValue(const T& value) {
  if (!this->type.isValid()) {
    this->type = DataType(typeid(T));
    
    if (this->type.isValid())
      this->value.reset(new ValueT<T>(value));
    else
      throw InvalidDataTypeException();
  }
  else if (typeid(T) == this->type.getTypeInfo()) {
    if (this->value)
      boost::static_pointer_cast<ValueT<T> >(this->value)->value = value;
    else
      this->value.reset(new ValueT<T>(value));
  }
  else
    throw DataTypeMismatchException(type.getIdentifier(),
      DataType(typeid(T)).getIdentifier());
}

template <typename T> T& Variant::getValue() {
  if (!this->type.isValid()) {
    this->type = DataType(typeid(T));
    
    if (this->type.isValid()) {
      this->value.reset(new ValueT<T>());
      
      return boost::static_pointer_cast<ValueT<T> >(this->value)->value;
    }
    else
      throw InvalidDataTypeException();
  }
  else if (typeid(T) == this->type.getTypeInfo()) {
    if (!this->value)
      this->value.reset(new ValueT<T>());
    
    return boost::static_pointer_cast<ValueT<T> >(this->value)->value;
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
        return boost::static_pointer_cast<ValueT<T> >(this->value)->value;
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
  return TypeTraits::EqualTo<T>::compare(
    static_cast<const ValueT<T>&>(value).value, this->value);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T, class Enable>
bool Variant::TypeTraits::EqualTo<T, Enable>::compare(const T& lhs, const
    T& rhs) {
  throw InvalidOperationException(
    "Comparing two variants of non-comparable type");
}

template <typename T>
bool Variant::TypeTraits::EqualTo<T, typename boost::enable_if<
    boost::has_equal_to<T, T, bool> >::type>::compare(const T& lhs, const
    T& rhs) {
  return (lhs == rhs);
}

template <typename T, class Enable>
void Variant::TypeTraits::ReadFrom<T, Enable>::read(std::istream& stream,
    T& value) {
  throw InvalidOperationException(
    "Reading a variant of non-readable type");
}

template <typename T>
void Variant::TypeTraits::ReadFrom<T, typename boost::enable_if<
    boost::has_right_shift<std::istream, T&> >::type>::read(std::istream&
    stream, T& value) {
  stream >> value;
}

template <typename T, class Enable>
void Variant::TypeTraits::WriteTo<T, Enable>::write(std::ostream& stream,
    const T& value) {
}

template <typename T>
void Variant::TypeTraits::WriteTo<T, typename boost::enable_if<
    boost::has_left_shift<std::ostream, const T&> >::type>::write(
    std::ostream& stream, const T& value) {
  stream << value;
}

template <typename T>
Variant::ValuePtr Variant::ValueT<T>::clone() const {
  return Variant::ValuePtr(new ValueT<T>(*this));
}

template <typename T>
void Variant::ValueT<T>::read(std::istream& stream) {
  TypeTraits::ReadFrom<T>::read(stream, this->value);
}

template <typename T>
void Variant::ValueT<T>::write(std::ostream& stream) const {
  TypeTraits::WriteTo<T>::write(stream, this->value);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

template <typename T> Variant& Variant::operator=(const T& value) {
  this->template setValue<T>(value);
  return *this;
}

template <typename T> Variant::operator const T&() const {
  return this->template getValue<T>();
}

}
