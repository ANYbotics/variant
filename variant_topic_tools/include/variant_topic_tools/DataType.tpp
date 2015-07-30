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

#include <boost/type_traits.hpp>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
DataType::ImplT<T>::ImplT(const std::string& identifier) :
  Impl(identifier) {
}

template <typename T>
DataType::ImplT<T>::~ImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
const std::type_info& DataType::ImplT<T>::getInfo() const {
  return typeid(T);
}

template <typename T>
size_t DataType::ImplT<T>::getSize() const {
  return ros::message_traits::template isFixedSize<T>() ? sizeof(T) : 0;
}

template <typename T>
bool DataType::ImplT<T>::isPrimitive() const {
  return !boost::is_compound<T>::value;
}

template <typename T>
bool DataType::ImplT<T>::isSimple() const {
  return ros::message_traits::template isSimple<T>();
}

template <typename T>
bool DataType::ImplT<T>::isFixedSize() const {
  return ros::message_traits::template isFixedSize<T>();
}

template <typename T>
bool DataType::ImplT<T>::isArithmetic() const {
  return boost::is_arithmetic<T>::value;
}

template <typename T>
bool DataType::ImplT<T>::isString() const {
  return boost::is_base_of<std::string, T>::value;
}

template <typename T>
bool DataType::ImplT<T>::isArray() const {
  return boost::is_array<T>::value;
}

template <typename T>
bool DataType::ImplT<T>::isMessage() const {
  return ros::message_traits::template IsMessage<T>::value;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> void DataType::Instances::add(const std::string&
    identifier) {
  DataType dataType;
  
  dataType.impl.reset(new ImplT<T>(identifier));
  insert(std::make_pair(dataType.impl->identifier, dataType));
}

template <typename T> void DataType::Instances::add() {
  this->template add<T>(ros::message_traits::template datatype<T>());
}

}
