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
#include <variant_topic_tools/ArrayVariant.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
ArrayDataType::ImplT<T>::ImplT() :
  Impl(typeid(typename type_traits::ArrayType<T>::MemberType)) {
}

template <typename T>
ArrayDataType::ImplT<T>::~ImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
const std::type_info& ArrayDataType::ImplT<T>::getTypeInfo() const {
  return typeid(T);
}

template <typename T>
size_t ArrayDataType::ImplT<T>::getNumMembers() const {
  return type_traits::ArrayType<T>::NumMembers;
}

template <typename T>
size_t ArrayDataType::ImplT<T>::getSize() const {
  return type_traits::ArrayType<T>::IsFixedSize::value ?
    sizeof(typename type_traits::ArrayType<T>::ValueType) : 0;
}

template <typename T>
bool ArrayDataType::ImplT<T>::isDynamic() const {
  return type_traits::ArrayType<T>::IsDynamic::value;
}

template <typename T>
bool ArrayDataType::ImplT<T>::isFixedSize() const {
  return type_traits::ArrayType<T>::IsFixedSize::value;
}

template <typename T>
bool ArrayDataType::ImplT<T>::isSimple() const {
  return type_traits::ArrayType<T>::IsSimple::value;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> ArrayDataType ArrayDataType::create() {
  ArrayDataType dataType;
  
  dataType.impl.reset(new boost::shared_ptr<DataType::Impl>(new ImplT<T>()));
  
  return dataType;
}

template <typename T, size_t N> ArrayDataType ArrayDataType::create(
    typename boost::enable_if<boost::type_traits::ice_eq<N, 0> >::type*) {
  return ArrayDataType::template create<T[]>();
}

template <typename T, size_t N> ArrayDataType ArrayDataType::create(
    typename boost::disable_if<boost::type_traits::ice_eq<N, 0> >::type*) {
  return ArrayDataType::template create<T[N]>();
}

template <typename T>
Serializer ArrayDataType::ImplT<T>::createSerializer(const DataType& type)
    const {
  return ArraySerializer::template create<T>();
}

template <typename T>
Variant ArrayDataType::ImplT<T>::createVariant(const DataType& type)
    const {
  return ArrayVariant::template create<T>(type, this->memberType);
}

}
