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

#include "variant_topic_tools/ArraySerializer.h"
#include <variant_topic_tools/ArrayTypeTraits.h>
#include <variant_topic_tools/ArrayVariant.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T, size_t N>
ArrayDataType::ImplT<T, N>::ImplT() :
  Impl(typeid(T)) {
}

template <typename T, size_t N>
ArrayDataType::ImplT<T, N>::~ImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T, size_t N>
const std::type_info& ArrayDataType::ImplT<T, N>::getTypeInfo() const {
  return typeid(typename ArrayTypeTraits::ToArray<T, N>::ArrayType);
}

template <typename T, size_t N>
size_t ArrayDataType::ImplT<T, N>::getNumElements() const {
  return N;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class A> ArrayDataType ArrayDataType::create() {
  return ArrayDataType::template create<typename
    ArrayTypeTraits::FromArray<A>::ElementType,
    ArrayTypeTraits::FromArray<A>::NumElements>();
}

template <typename T, size_t N> ArrayDataType ArrayDataType::create() {
  ArrayDataType dataType;
  
  dataType.impl.reset(new boost::shared_ptr<DataType::Impl>(
    new ImplT<T, N>()));
  
  return dataType;
}

template <typename T, size_t N>
Serializer ArrayDataType::ImplT<T, N>::createSerializer(const DataType& type)
    const {
  return ArraySerializer::template create<T, N>();
}

template <typename T, size_t N>
Variant ArrayDataType::ImplT<T, N>::createVariant(const DataType& type)
    const {
  return ArrayVariant::template create<T, N>(type, this->elementType);
}

}
