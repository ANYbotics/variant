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

/** \file DataTypeTraits.h
 * \brief Header file providing the data type traits
 */

#ifndef VARIANT_TOPIC_TOOLS_DATA_TYPE_TRAITS_H
#define VARIANT_TOPIC_TOOLS_DATA_TYPE_TRAITS_H

#include <boost/type_traits.hpp>

#include <variant_topic_tools/ArrayTypeTraits.h>
#include <variant_topic_tools/BuiltinTypeTraits.h>
#include <variant_topic_tools/MessageTypeTraits.h>

namespace variant_topic_tools {
namespace type_traits {
template <typename T, typename Enable = void>
struct DataType;

template <typename T>
struct DataType<T, typename boost::enable_if<IsBuiltin<T> >::type> {
  using ValueType = typename BuiltinType<T>::ValueType;
  using IsFixedSize = typename BuiltinType<T>::IsFixedSize;
  using IsSimple = typename BuiltinType<T>::IsFixedSize;
};

template <typename T>
struct DataType<T, typename boost::enable_if<IsArray<T> >::type> {
  using ValueType = typename ArrayType<T>::ValueType;
  using IsFixedSize = typename ArrayType<T>::IsFixedSize;
  using IsSimple = typename ArrayType<T>::IsFixedSize;
};

template <typename T>
struct DataType<T, typename boost::enable_if<IsMessage<T> >::type> {
  using ValueType = typename MessageType<T>::ValueType;
  using IsFixedSize = typename MessageType<T>::IsFixedSize;
  using IsSimple = typename MessageType<T>::IsFixedSize;
};

template <typename T, typename Enable = void>
struct ToDataType;

template <typename T>
struct ToDataType<T, typename boost::enable_if<IsBuiltin<typename ToBuiltinType<T>::BuiltinType> >::type> {
  using DataType = typename ToBuiltinType<T>::BuiltinType;
};

template <typename T>
struct ToDataType<T, typename boost::enable_if<IsArray<typename ToArrayType<T>::ArrayType> >::type> {
  using DataType = typename ToArrayType<T>::ArrayType;
};

template <typename T>
struct ToDataType<T, typename boost::enable_if<IsMessage<typename ToMessageType<T>::MessageType> >::type> {
  using DataType = typename ToMessageType<T>::MessageType;
};
}  // namespace type_traits
}  // namespace variant_topic_tools

#endif
