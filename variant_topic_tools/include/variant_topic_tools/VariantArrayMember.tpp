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

#include <variant_topic_tools/Exceptions.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T, size_t N>
VariantArrayMember::ValueImplT<T, N>::ValueImplT(const ArrayPtr& array,
    size_t index) :
  array(array),
  index(index) {
}

template <typename T, size_t N>
VariantArrayMember::ValueImplT<T, N>::ValueImplT(const ValueImplT<T, N>& src) :
  array(src.array),
  index(src.index) {
}

template <typename T, size_t N>
VariantArrayMember::ValueImplT<T, N>::~ValueImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T, size_t N>
size_t VariantArrayMember::TypeTraits::ToArray<T, N>::getSize(ArrayType&
    array) {
  return N;
}

template <typename T>
size_t VariantArrayMember::TypeTraits::ToArray<T, 0>::getSize(ArrayType&
    array) {
  return array.size();
}

template <typename T, size_t N>
void VariantArrayMember::ValueImplT<T, N>::setValue(const T& value) {
  ArrayPtr array = this->array.lock();
  
  if (array && (this->index < VariantArrayMember::TypeTraits::
      ToArray<T, N>::getSize(*array)))
    (*array)[this->index] = value;
  else
    throw NoSuchMemberException(index);
}

template <typename T, size_t N>
T& VariantArrayMember::ValueImplT<T, N>::getValue() {
  ArrayPtr array = this->array.lock();
  
  if (array && (this->index < VariantArrayMember::TypeTraits::
      ToArray<T, N>::getSize(*array)))
    return (*array)[this->index];
  else
    throw NoSuchMemberException(index);
}

template <typename T, size_t N>
const T& VariantArrayMember::ValueImplT<T, N>::getValue() const {
  ArrayPtr array = this->array.lock();
  
  if (array && (this->index < VariantArrayMember::TypeTraits::
      ToArray<T, N>::getSize(*array)))
    return (*array)[this->index];
  else
    throw NoSuchMemberException(index);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T, size_t N>
VariantArrayMember VariantArrayMember::create(const DataType& type, const
    typename VariantArrayMember::ValueImplT<T, N>::ArrayPtr& array, size_t
    index) {
  VariantArrayMember variantArrayMember;
  
  variantArrayMember.type = type;
  variantArrayMember.value.reset(new ValueImplT<T, N>(array, index));
  
  return variantArrayMember;
}

template <typename T, size_t N>
Variant::ValuePtr VariantArrayMember::ValueImplT<T, N>::clone() const {
  return Variant::ValuePtr(new ValueImplT<T, N>(*this));
}

}
