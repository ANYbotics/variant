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

#include <variant_topic_tools/ArrayDataType.h>
#include <variant_topic_tools/ArrayMemberPointer.h>
#include <variant_topic_tools/BuiltinPointer.h>
#include <variant_topic_tools/Exceptions.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T, size_t N>
ArrayVariant::ValueImplT<T, N>::ValueImplT(const DataType& memberType, const
    Pointer<ArrayType>& array) :
  memberType(memberType),
  array(array) {
}

template <typename T, size_t N>
ArrayVariant::ValueImplT<T, N>::ValueImplT(const ValueImplT<T, N>& src) :
  memberType(src.memberType),
  array(src.array) {
}

template <typename T, size_t N>
ArrayVariant::ValueImplT<T, N>::~ValueImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T, size_t N>
void ArrayVariant::ValueImplT<T, N>::set(const Pointer<ArrayType>& value) {
  this->array = value;
}

template <typename T, size_t N>
void ArrayVariant::ValueImplT<T, N>::setValue(const ArrayType& value) {
  if (!this->array) {
    this->array = BuiltinPointer<ArrayType>(new ArrayType());
    ArrayVariant::template initialize<T, N>(*this->array);
  }
    
  *this->array = value;
}

template <typename T, size_t N>
typename ArrayVariant::ValueImplT<T, N>::ArrayType&
    ArrayVariant::ValueImplT<T, N>::getValue() {
  if (!this->array) {
    this->array = BuiltinPointer<ArrayType>(new ArrayType());
    ArrayVariant::template initialize<T, N>(*this->array);
  }
  
  return *this->array;
}

template <typename T, size_t N>
const typename ArrayVariant::ValueImplT<T, N>::ArrayType&
    ArrayVariant::ValueImplT<T, N>::getValue() const {
  if (!this->array) {
    static ArrayType array = ArrayType();
    static bool initialized = false;
    
    if (!initialized) {
      ArrayVariant::template initialize<T, N>(array);
      initialized = true;
    }
    
    return array;
  }
  else
    return *this->array;
}

template <typename T, size_t N>
size_t ArrayVariant::ValueImplT<T, N>::getNumMembers() const {
  if (this->array)
    return this->array->size();
  else
    return 0;
}

template <typename T, size_t N>
void ArrayVariant::ValueImplT<T, N>::setMember(size_t index, const Variant&
    member) {
  if (!this->array) {
    this->array = BuiltinPointer<ArrayType>(new ArrayType());
    ArrayVariant::template initialize<T, N>(*this->array);
  }
  
  if (index < this->array->size())
    (*this->array)[index] = member.template getValue<T>();
  else
    throw NoSuchMemberException(index);
}

template <typename T, size_t N>
Variant ArrayVariant::ValueImplT<T, N>::getMember(size_t index) const {
  if (!this->array) {
    this->array = BuiltinPointer<ArrayType>(new ArrayType());
    ArrayVariant::template initialize<T, N>(*this->array);
  }
  
  if (index < this->array->size()) {
    Variant member = this->memberType.createVariant();

    Variant::template set<T>(member,
      ArrayMemberPointer<ArrayType, T>(this->array, index));
    
    return member;
  }
  else
    throw NoSuchMemberException(index);  
}

template <typename T, size_t N>
bool ArrayVariant::ValueImplT<T, N>::isFixedSize() const {
  return N;
}

template <typename T, size_t N>
bool ArrayVariant::ValueImplT<T, N>::isEqual(const Variant::Value& value)
    const {
  return CollectionVariant::Value::isEqual(value);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T, size_t N> ArrayVariant ArrayVariant::create(const
    DataType& type, const DataType& memberType) {
  ArrayVariant variant;
  
  variant.type = type;
  variant.value.reset(new ValueImplT<T, N>(memberType));
  
  return variant;
}

template <typename T, size_t N>
void ArrayVariant::ValueImplT<T, N>::addMember(const Variant& member) {
  if (!this->array) {
    this->array = BuiltinPointer<ArrayType>(new ArrayType());
    ArrayVariant::template initialize<T, N>(*this->array);
  }
  
  ArrayVariant::template add<T, N>(*this->array, member);
}

template <typename T, size_t N>
void ArrayVariant::ValueImplT<T, N>::resize(size_t numMembers) {
  if (!this->array) {
    this->array = BuiltinPointer<ArrayType>(new ArrayType());
    ArrayVariant::template initialize<T, N>(*this->array);
  }
  
  ArrayVariant::template resize<T, N>(*this->array, numMembers);
}

template <typename T, size_t N>
void ArrayVariant::ValueImplT<T, N>::clear() {
  if (this->array)
    ArrayVariant::template clear<T, N>(*this->array);
}

template <typename T, size_t N>
Variant::ValuePtr ArrayVariant::ValueImplT<T, N>::clone() const {
  return Variant::ValuePtr(new ValueImplT<T, N>(*this));
}

template <typename T, size_t N> void ArrayVariant::initialize(typename
    ArrayTypeTraits::ToArray<T, N>::ArrayType& array, typename boost::
    disable_if<boost::type_traits::ice_eq<N, 0> >::type*) {
  array.assign(T());
}

template <typename T, size_t N> void ArrayVariant::initialize(typename
    ArrayTypeTraits::ToArray<T, N>::ArrayType& array, typename boost::
    enable_if<boost::type_traits::ice_eq<N, 0> >::type*) {
}

template <typename T, size_t N> void ArrayVariant::add(typename
    ArrayTypeTraits::ToArray<T, N>::ArrayType& array, const T& element,
    typename boost::disable_if<boost::type_traits::ice_eq<N, 0> >::type*) {
  throw InvalidOperationException("Adding a member to a fixed-size array");
}

template <typename T, size_t N> void ArrayVariant::add(typename
    ArrayTypeTraits::ToArray<T, N>::ArrayType& array, const T& element,
    typename boost::enable_if<boost::type_traits::ice_eq<N, 0> >::type*) {
  array.push_back(element); 
}

template <typename T, size_t N> void ArrayVariant::resize(typename
    ArrayTypeTraits::ToArray<T, N>::ArrayType& array, size_t numElements,
    typename boost::disable_if<boost::type_traits::ice_eq<N, 0> >::type*) {
  if (numElements != N)
    throw InvalidOperationException("Resizing a fixed-size array");
}

template <typename T, size_t N> void ArrayVariant::resize(typename
    ArrayTypeTraits::ToArray<T, N>::ArrayType& array, size_t numElements,
    typename boost::enable_if<boost::type_traits::ice_eq<N, 0> >::type*) {
  array.resize(numElements);
}

template <typename T, size_t N> void ArrayVariant::clear(typename
    ArrayTypeTraits::ToArray<T, N>::ArrayType& array, typename boost::
    disable_if<boost::type_traits::ice_eq<N, 0> >::type*) {
  throw InvalidOperationException("Clearing a fixed-size array");
}

template <typename T, size_t N> void ArrayVariant::clear(typename
    ArrayTypeTraits::ToArray<T, N>::ArrayType& array, typename boost::
    enable_if<boost::type_traits::ice_eq<N, 0> >::type*) {
  array.clear();
}

template <typename T, size_t N>
void ArrayVariant::ValueImplT<T, N>::read(std::istream& stream) {
  CollectionVariant::Value::read(stream);
}

template <typename T, size_t N>
void ArrayVariant::ValueImplT<T, N>::write(std::ostream& stream) const {
  CollectionVariant::Value::write(stream);
}

}
