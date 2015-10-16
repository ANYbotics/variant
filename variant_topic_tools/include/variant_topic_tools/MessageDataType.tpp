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
#include <variant_topic_tools/MessageDefinition.h>
#include <variant_topic_tools/MessageSerializer.h>
#include <variant_topic_tools/MessageStream.h>
#include <variant_topic_tools/MessageVariant.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
MessageDataType::ImplT<T>::ImplT() :
  Impl(ros::message_traits::template definition<T>()) {
  T message;
  MessageStream stream(message);
  
  ros::serialization::serialize(stream, message);
  
  size_t numConstants = 0;
  for (size_t i = 0; i < this->members.size(); ++i) {
    if (this->members[i].isVariable()) {
      BOOST_ASSERT(i-numConstants < stream.getNumMembers());
      
      boost::static_pointer_cast<MessageVariable::Impl>(
        this->members[i].impl)->offset = stream.getMemberOffset(
          i-numConstants);
    }
    else
      ++numConstants;
  }
  
  BOOST_ASSERT(this->members.size()-numConstants == stream.getNumMembers());
}

template <typename T>
MessageDataType::ImplT<T>::~ImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
const std::string& MessageDataType::ImplT<T>::getIdentifier() const {
  static std::string identifier(ros::message_traits::template datatype<T>());
  return identifier;
}

template <typename T>
const std::type_info& MessageDataType::ImplT<T>::getTypeInfo() const {
  return typeid(T);
}

template <typename T>
size_t MessageDataType::ImplT<T>::getSize() const {
  return ros::message_traits::template isFixedSize<T>() ? sizeof(T) : 0;
}

template <typename T>
const std::string& MessageDataType::ImplT<T>::getMD5Sum() const {
  static std::string md5Sum(ros::message_traits::template md5sum<T>());
  return md5Sum;
}

template <typename T>
const std::string& MessageDataType::ImplT<T>::getDefinition() const {
  static std::string definition(ros::message_traits::template definition<T>());
  return definition;
}

template <typename T>
bool MessageDataType::ImplT<T>::isSimple() const {
  return ros::message_traits::template isSimple<T>();
}

template <typename T>
bool MessageDataType::ImplT<T>::isFixedSize() const {
  return ros::message_traits::template isFixedSize<T>();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> MessageDataType MessageDataType::create() {
  MessageDefinition::template create<T>();
  MessageDataType dataType;
  
  dataType.impl.reset(new boost::shared_ptr<DataType::Impl>(new ImplT<T>()));
  
  return dataType;
}

template <typename T> MessageConstant MessageDataType::addConstant(const
    std::string& name, const T& value) {
  return this->addConstant(name, Variant(value));
}

template <typename T> MessageVariable MessageDataType::addVariable(const
    std::string& name) {
  return this->addVariable(name, typeid(T));
}

template <typename T>
Serializer MessageDataType::ImplT<T>::createSerializer(const DataType& type)
    const {
  return MessageSerializer::template create<T>();
}

template <typename T>
Variant MessageDataType::ImplT<T>::createVariant(const DataType& type) const {
  return static_cast<const Variant&>(MessageVariant::template create<T>(type,
    this->members));
}

template <typename T>
void MessageDataType::ImplT<T>::addMember(const MessageMember& member) {
  throw ImmutableDataTypeException();
}

}
