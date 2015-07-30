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
Variant::ValueT<T>::ValueT(const T& invariant) :
  invariant(invariant) {
}

template <typename T>
Variant::ValueT<T>::~ValueT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T> void Variant::setValue(const T& value) {
}

template <typename T> const T& Variant::getValue() const {
  if (value) {
    if (typeid(T) == type.getInfo())
      return boost::static_pointer_cast<ValueT<T> >(value)->invariant;
    else
      throw DataTypeMismatchException(type.getIdentifier(), "unresolved");
  }
  else
    throw InvalidDataTypeException("void");
}

template <typename T>
bool Variant::ValueT<T>::isEqual(const Value& value) const {
  return (static_cast<const ValueT<T>&>(value).invariant == invariant);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T>
Variant::ValuePtr Variant::ValueT<T>::clone() const {
  return new ValueT<T>(invariant);
}

template <typename T>
void Variant::ValueT<T>::write(std::ostream& stream) const {
  stream << invariant;
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
