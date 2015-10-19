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
  
  BOOST_ASSERT(this->variableMembers.size() == stream.getNumMembers());
  for (size_t i = 0; i < this->variableMembers.size(); ++i) {
//     BOOST_ASSERT(this->variableMembers[i].getType().getTypeInfo() == 
//       stream.getMemberTypeInfo(i));
    
    boost::static_pointer_cast<MessageVariable::Impl>(
      this->variableMembers[i].impl)->offset = stream.getMemberOffset(i);
  }
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

template <typename T> MessageConstant MessageDataType::addConstantMember(const
    std::string& name, const T& value) {
  return this->addConstantMember(name, Variant(value));
}

template <typename T> MessageVariable MessageDataType::addVariableMember(const
    std::string& name) {
  return this->addVariableMember(name, typeid(T));
}

template <typename T>
Serializer MessageDataType::ImplT<T>::createSerializer(const DataType& type)
    const {
  return MessageSerializer::template create<T>();
}

template <typename T>
Variant MessageDataType::ImplT<T>::createVariant(const DataType& type) const {
  return static_cast<const Variant&>(MessageVariant::template create<T>(type,
    this->variableMembers));
}

template <typename T>
void MessageDataType::ImplT<T>::addConstantMember(const MessageConstant&
    member) {
  throw ImmutableDataTypeException();
}

template <typename T>
void MessageDataType::ImplT<T>::addVariableMember(const MessageVariable&
    member) {
  throw ImmutableDataTypeException();
}

}
