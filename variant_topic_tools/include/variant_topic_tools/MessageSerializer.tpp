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
MessageSerializer::ImplT<T>::ImplT(const DataType& dataType) :
  MessageSerializer::Impl(dataType) {
  
  boost::shared_ptr<T> invariant(new T());
  std::vector<uint8_t> data;
  ros::serialization::IStream stream(data.data(), 1);
  ros::serialization::deserialize(stream, *invariant);
}

template <typename T>
MessageSerializer::ImplT<T>::~ImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
size_t MessageSerializer::ImplT<T>::getSerializedLength(Variant& value)
    const {
  return ros::serialization::serializationLength(value.template getValue<T>());
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> MessageSerializer MessageSerializer::createSimple(const
    DataType& dataType) {
  MessageSerializer serializer(dataType);
  
  if (!serializer.impl) {
    serializer.impl.reset(new ImplT<T>(dataType));
    getInstances().insert(std::make_pair(dataType, serializer));
  }
  
  return serializer;
}

template <typename T>
void MessageSerializer::ImplT<T>::read(ros::serialization::IStream& stream,
    Variant& value) {
  T invariant;
  
  ros::serialization::deserialize(stream, invariant);
  
  value.template setValue<T>(invariant);
}

template <typename T>
void MessageSerializer::ImplT<T>::write(ros::serialization::OStream& stream,
    const Variant& value) {
  ros::serialization::serialize(stream, value.template getValue<T>());
}

}
