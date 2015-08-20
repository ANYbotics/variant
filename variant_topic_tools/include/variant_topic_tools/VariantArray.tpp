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
#include <variant_topic_tools/Exceptions.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T, size_t N>
VariantArray::ValueT<T, N>::ValueT(const ArrayType & members) :
  members(members) {
}

template <typename T, size_t N>
VariantArray::ValueT<T, N>::ValueT(const ValueT<T, N>& src) :
  members(src.members) {
}

template <typename T, size_t N>
VariantArray::ValueT<T, N>::~ValueT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T, size_t N>
size_t VariantArray::ValueT<T, N>::getNumMembers() const {
  return members.size();
}

template <typename T, size_t N>
Variant& VariantArray::ValueT<T, N>::getMember(size_t index) {
  if (index < this->members.size())
    return this->members[index];
  else
    throw NoSuchMemberException(index);
}

template <typename T, size_t N>
const Variant& VariantArray::ValueT<T, N>::getMember(size_t index) const {
  if (index < this->members.size())
    return this->members[index];
  else
    throw NoSuchMemberException(index);
}

template <typename T, size_t N>
bool VariantArray::ValueT<T, N>::isFixedSize() const {
  return N;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T, size_t N> VariantArray VariantArray::create() {
  VariantArray variantArray;
  
  variantArray.type = ArrayDataType::template create<T, N>();
  variantArray.value.reset(new ValueT<T, N>());
  
  return variantArray;
}

template <typename T, size_t N>
void VariantArray::TypeTraits::ToArray<T, N>::add(ArrayType& array,
    const T& element) {
  throw InvalidOperationException("Adding a member to a fixed-size array");
}

template <typename T>
void VariantArray::TypeTraits::ToArray<T, 0>::add(ArrayType& array,
    const T& element) {
  array.push_back(element); 
}

template <typename T, size_t N>
void VariantArray::ValueT<T, N>::addMember(const Variant& member) {
  VariantArray::TypeTraits::ToArray<T, N>::add(members, member);
}

template <typename T, size_t N>
void VariantArray::TypeTraits::ToArray<T, N>::resize(ArrayType& array,
    size_t numElements) {
  if (numElements != N)
    throw InvalidOperationException("Resizing a fixed-size array");
}

template <typename T>
void VariantArray::TypeTraits::ToArray<T, 0>::resize(ArrayType& array,
     size_t numElements) {
  array.resize(numElements, Variant(T())); 
}

template <typename T, size_t N>
void VariantArray::ValueT<T, N>::resize(size_t numMembers) {
  VariantArray::TypeTraits::ToArray<T, N>::resize(members, numMembers);
}

template <typename T, size_t N>
void VariantArray::TypeTraits::ToArray<T, N>::clear(ArrayType& array) {
  throw InvalidOperationException("Clearing a fixed-size array");
}

template <typename T>
void VariantArray::TypeTraits::ToArray<T, 0>::clear(ArrayType& array) {
  array.clear();
}

template <typename T, size_t N>
void VariantArray::ValueT<T, N>::clear() {
  VariantArray::TypeTraits::ToArray<T, N>::clear(members);
}

template <typename T, size_t N>
Variant::ValuePtr VariantArray::ValueT<T, N>::clone() const {
  return Variant::ValuePtr(new ValueT<T, N>(*this));
}

}
