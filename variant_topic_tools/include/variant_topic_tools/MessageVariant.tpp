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

#include <variant_topic_tools/MessageMemberPointer.h>
#include <variant_topic_tools/Exceptions.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
MessageVariant::ValueImplT<T>::ValueImplT(const std::vector<MessageMember>&
    members, const Pointer<ValueType>& message) :
  message(message) {
  for (size_t i = 0; i < members.size(); ++i)
    this->members.appendField(members[i].getName(), members[i]);
}

template <typename T>
MessageVariant::ValueImplT<T>::ValueImplT(const ValueImplT<T>& src) :
  members(src.members),
  message(src.message) {
}

template <typename T>
MessageVariant::ValueImplT<T>::~ValueImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
void MessageVariant::ValueImplT<T>::set(const Pointer<ValueType>& value) {
  this->message = value;
}
      
template <typename T>
typename MessageVariant::ValueImplT<T>::ValueType& MessageVariant::
    ValueImplT<T>::getValue() {
  if (!this->message)
    this->message = Pointer<ValueType>(new ValueType());
  
  return *this->message;
}

template <typename T>
const typename MessageVariant::ValueImplT<T>::ValueType& MessageVariant::
    ValueImplT<T>::getValue() const {
  if (!this->message) {
    static ValueType message = ValueType();
    return message;
  }
  else
    return *this->message;
}

template <typename T>
size_t MessageVariant::ValueImplT<T>::getNumMembers() const {
  return this->members.getNumFields();
}

template <typename T>
void MessageVariant::ValueImplT<T>::setMember(size_t index, const Variant&
    member) {
  this->getMember(index).setValue(member);
}

template <typename T>
void MessageVariant::ValueImplT<T>::setMember(const std::string& name,
    const Variant& member) {
  this->getMember(name).setValue(member);
}

template <typename T>
Variant MessageVariant::ValueImplT<T>::getMember(size_t index) const {
  if (!this->message)
    this->message = Pointer<ValueType>(new ValueType());
  
  Variant member = this->members[index].getValue().getType().createVariant();
//   Variant::template set<T>(member,
//     MessageMemberPointer<ArrayType, T>(this->message, index));
  
  return member;
}

template <typename T>
Variant MessageVariant::ValueImplT<T>::getMember(const std::string& name)
    const {
  if (!this->message)
    this->message = Pointer<ValueType>(new ValueType());
  
  Variant member = this->members[name].getValue().getType().createVariant();
//   Variant::template set<T>(member,
//     MessageMemberPointer<ArrayType, T>(this->message, index));
  
  return member;
}

template <typename T>
const std::string& MessageVariant::ValueImplT<T>::getMemberName(size_t
    index) const {
  return this->members.getField(index).getName();
}

template <typename T>
bool MessageVariant::ValueImplT<T>::hasMember(const std::string& name)
    const {
  return this->members.hasField(name);
}

template <typename T>
bool MessageVariant::ValueImplT<T>::isEqual(const Variant::Value& value)
    const {
  return CollectionVariant::Value::isEqual(value);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> MessageVariant MessageVariant::create(const DataType&
    type, const std::vector<MessageMember>& members) {
  MessageVariant variant;
  
  variant.type = type;
  variant.value.reset(new ValueImplT<T>(members));
    
  return variant;
}

template <typename T>
Variant::ValuePtr MessageVariant::ValueImplT<T>::clone() const {
  return Variant::ValuePtr(new ValueImplT<T>(*this));
}

template <typename T>
void MessageVariant::ValueImplT<T>::read(std::istream& stream) {
  CollectionVariant::Value::read(stream);
}

template <typename T>
void MessageVariant::ValueImplT<T>::write(std::ostream& stream) const {
  CollectionVariant::Value::write(stream);
}

}
