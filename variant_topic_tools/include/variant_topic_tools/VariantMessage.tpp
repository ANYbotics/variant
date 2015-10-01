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

#include <variant_topic_tools/MessageDataType.h>
#include <variant_topic_tools/Exceptions.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
VariantMessage::ValueImplT<T>::ValueImplT(const MessageFieldCollection<
  std::pair<DataType, size_t> > memberTypesAndOffsets, const T& members) :
  memberTypesAndOffsets(memberTypesAndOffsets),
  members(new T(members)) {
}

template <typename T>
VariantMessage::ValueImplT<T>::ValueImplT(const ValueImplT<T>& src) :
  memberTypesAndOffsets(src.memberTypesAndOffsets),
  members(new T(*src.members)) {
}

template <typename T>
VariantMessage::ValueImplT<T>::~ValueImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
void VariantMessage::ValueImplT<T>::setValue(const T& value) {
  *this->members = value;
}

template <typename T>
T& VariantMessage::ValueImplT<T>::getValue() {
  return *this->members;
}

template <typename T>
const T& VariantMessage::ValueImplT<T>::getValue() const {
  return *this->members;
}

template <typename T>
size_t VariantMessage::ValueImplT<T>::getNumMembers() const {
  return this->memberTypesAndOffsets.getNumFields();
}

template <typename T>
void VariantMessage::ValueImplT<T>::setMember(size_t index, const
    Variant& member) {
//   members.getField(index).setValue(member);
}

template <typename T>
void VariantMessage::ValueImplT<T>::setMember(const std::string& name,
    const Variant& member) {
//   members.getField(name).setValue(member);
}

template <typename T>
SharedVariant VariantMessage::ValueImplT<T>::getMember(size_t index)
    const {
  return this->memberTypesAndOffsets[index].getValue().first.createVariant();
}

template <typename T>
SharedVariant VariantMessage::ValueImplT<T>::getMember(const std::string&
    name) const {
  return this->memberTypesAndOffsets[name].getValue().first.createVariant();
}

template <typename T>
const std::string& VariantMessage::ValueImplT<T>::getMemberName(size_t
    index) const {
  return this->memberTypesAndOffsets.getField(index).getName();
}

template <typename T>
bool VariantMessage::ValueImplT<T>::hasMember(const std::string& name)
    const {
  return this->memberTypesAndOffsets.hasField(name);
}

template <typename T>
bool VariantMessage::ValueImplT<T>::isEqual(const Variant::Value& value)
    const {
  return VariantCollection::Value::isEqual(value);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> VariantMessage VariantMessage::create() {
  VariantMessage variantMessage;
  MessageDataType type = MessageDataType::template create<T>(); 
  MessageFieldCollection<std::pair<DataType, size_t> > memberTypesAndOffsets;
  
  for (size_t i = 0; i < type.getNumMembers(); ++i)
    memberTypesAndOffsets.appendField(type.getMember(i).getName(),
      std::make_pair(type.getMember(i).getType(), 0));
  
  variantMessage.type = type,
  variantMessage.value.reset(new ValueImplT<T>(memberTypesAndOffsets));
  
  return variantMessage;
}

template <typename T>
Variant::ValuePtr VariantMessage::ValueImplT<T>::clone() const {
  return Variant::ValuePtr(new ValueImplT<T>(*this));
}

template <typename T>
void VariantMessage::ValueImplT<T>::read(std::istream& stream) {
  VariantCollection::Value::read(stream);
}

template <typename T>
void VariantMessage::ValueImplT<T>::write(std::ostream& stream) const {
  VariantCollection::Value::write(stream);
}

}
