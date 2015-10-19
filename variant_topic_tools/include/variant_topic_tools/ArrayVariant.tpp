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

#include <variant_topic_tools/ArrayMemberPointer.h>
#include <variant_topic_tools/Exceptions.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
ArrayVariant::ValueImplT<T>::ValueImplT(const DataType& memberType, const
    Pointer<ValueType>& array) :
  memberType(memberType),
  array(array) {
}

template <typename T>
ArrayVariant::ValueImplT<T>::ValueImplT(const ValueImplT<T>& src) :
  memberType(src.memberType),
  array(src.array) {
}

template <typename T>
ArrayVariant::ValueImplT<T>::~ValueImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
void ArrayVariant::ValueImplT<T>::set(const Pointer<ValueType>& value) {
  this->array = value;
}

template <typename T>
typename ArrayVariant::ValueImplT<T>::ValueType& ArrayVariant::
    ValueImplT<T>::getValue() {
  if (!this->array) {
    this->array = Pointer<ValueType>(new ValueType());
    ArrayVariant::template initialize<T>(*this->array);
  }
  
  return *this->array;
}

template <typename T>
const typename ArrayVariant::ValueImplT<T>::ValueType& ArrayVariant::
    ValueImplT<T>::getValue() const {
  if (!this->array) {
    static ValueType array = ValueType();
    static bool initialized = false;
    
    if (!initialized) {
      ArrayVariant::template initialize<T>(array);
      initialized = true;
    }
    
    return array;
  }
  else
    return *this->array;
}

template <typename T>
size_t ArrayVariant::ValueImplT<T>::getNumMembers() const {
  if (this->array)
    return this->array->size();
  else
    return 0;
}

template <typename T>
void ArrayVariant::ValueImplT<T>::setMember(size_t index, const Variant&
    member) {
  if (!this->array) {
    this->array = Pointer<ValueType>(new ValueType());
    ArrayVariant::template initialize<T>(*this->array);
  }
  
  if (index < this->array->size())
    (*this->array)[index] = member.template getValue<MemberType>();
  else
    throw NoSuchMemberException(index);
}

template <typename T>
Variant ArrayVariant::ValueImplT<T>::getMember(size_t index) const {
  if (!this->array) {
    this->array = Pointer<ValueType>(new ValueType());
    ArrayVariant::template initialize<T>(*this->array);
  }
  
  if (index < this->array->size()) {
    Variant member = this->memberType.createVariant();

    Variant::template set<MemberType>(member, ArrayMemberPointer<T>(
      this->array, index));
    
    return member;
  }
  else
    throw NoSuchMemberException(index);  
}

template <typename T>
bool ArrayVariant::ValueImplT<T>::isFixedSize() const {
  return NumMembers;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> ArrayVariant ArrayVariant::create(const
    DataType& type, const DataType& memberType) {
  ArrayVariant variant;
  
  variant.type = type;
  variant.value.reset(new ValueImplT<T>(memberType));
  
  return variant;
}

template <typename T>
void ArrayVariant::ValueImplT<T>::addMember(const Variant& member) {
  if (!this->array) {
    this->array = Pointer<ValueType>(new ValueType());
    ArrayVariant::template initialize<T>(*this->array);
  }
  
  ArrayVariant::template add<T>(*this->array, member);
}

template <typename T>
void ArrayVariant::ValueImplT<T>::resize(size_t numMembers) {
  if (!this->array) {
    this->array = Pointer<ValueType>(new ValueType());
    ArrayVariant::template initialize<T>(*this->array);
  }
  
  ArrayVariant::template resize<T>(*this->array, numMembers);
}

template <typename T>
void ArrayVariant::ValueImplT<T>::clear() {
  if (this->array)
    ArrayVariant::template clear<T>(*this->array);
}

template <typename T>
Variant::ValuePtr ArrayVariant::ValueImplT<T>::clone() const {
  return Variant::ValuePtr(new ValueImplT<T>(*this));
}

template <typename T> void ArrayVariant::initialize(typename type_traits::
    ArrayType<T>::ValueType& array, typename boost::enable_if<typename
    type_traits::ArrayType<T>::IsFixedSize>::type*) {
  array.assign(typename type_traits::ArrayType<T>::MemberValueType());
}

template <typename T> void ArrayVariant::initialize(typename type_traits::
    ArrayType<T>::ValueType& array, typename boost::disable_if<typename
    type_traits::ArrayType<T>::IsFixedSize>::type*) {
}

template <typename T> void ArrayVariant::add(typename type_traits::
    ArrayType<T>::ValueType& array, const typename type_traits::
    ArrayType<T>::MemberType& member, typename boost::enable_if<
    typename type_traits::ArrayType<T>::IsFixedSize>::type*) {
  throw InvalidOperationException("Adding a member to a fixed-size array");
}

template <typename T> void ArrayVariant::add(typename type_traits::
    ArrayType<T>::ValueType& array, const typename type_traits::
    ArrayType<T>::MemberType& member, typename boost::disable_if<
    typename type_traits::ArrayType<T>::IsFixedSize>::type*) {
  array.push_back(member); 
}

template <typename T> void ArrayVariant::resize(typename type_traits::
    ArrayType<T>::ValueType& array, size_t numMembers, typename boost::
    enable_if<typename type_traits::ArrayType<T>::IsFixedSize>::type*) {
  if (numMembers != type_traits::ArrayType<T>::NumMembers)
    throw InvalidOperationException("Resizing a fixed-size array");
}

template <typename T> void ArrayVariant::resize(typename type_traits::
    ArrayType<T>::ValueType& array, size_t numMembers, typename boost::
    disable_if<typename type_traits::ArrayType<T>::IsFixedSize>::type*) {
  array.resize(numMembers);
}

template <typename T> void ArrayVariant::clear(typename type_traits::
    ArrayType<T>::ValueType& array, typename boost::enable_if<typename
    type_traits::ArrayType<T>::IsFixedSize>::type*) {
  throw InvalidOperationException("Clearing a fixed-size array");
}

template <typename T> void ArrayVariant::clear(typename type_traits::
    ArrayType<T>::ValueType& array, typename boost::disable_if<typename
    type_traits::ArrayType<T>::IsFixedSize>::type*) {
  array.clear();
}

}
