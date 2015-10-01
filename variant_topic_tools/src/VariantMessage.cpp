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

#include "variant_topic_tools/MessageDataType.h"
#include "variant_topic_tools/VariantMessage.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

VariantMessage::VariantMessage() {
}

VariantMessage::VariantMessage(const MessageDataType& type, const
    MessageFieldCollection<Variant>& members) :
  VariantCollection(static_cast<const DataType&>(type)) {
  if (type.isValid())
    value.reset(new ValueImplV(members));
}

VariantMessage::VariantMessage(const VariantMessage& src) :
  VariantCollection(src) {
}

VariantMessage::VariantMessage(const Variant& src) :
  VariantCollection(src) {
  if (value)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Value>(value));
}

VariantMessage::~VariantMessage() {
}

VariantMessage::Value::Value() {
}

VariantMessage::Value::~Value() {
}

VariantMessage::ValueImplV::ValueImplV(const MessageFieldCollection<Variant>&
    members) :
  members(members) {
}

VariantMessage::ValueImplV::ValueImplV(const ValueImplV& src) :
  members(src.members) {
}

VariantMessage::ValueImplV::~ValueImplV() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t VariantMessage::ValueImplV::getNumMembers() const {
  return members.getNumFields();
}

void VariantMessage::ValueImplV::setMember(size_t index, const Variant&
    member) {
  members.getField(index).setValue(member);
}

void VariantMessage::ValueImplV::setMember(const std::string& name, const
    Variant& member) {
  members.getField(name).setValue(member);
}

SharedVariant VariantMessage::ValueImplV::getMember(size_t index) const {
  return members.getField(index).getValue();
}

SharedVariant VariantMessage::ValueImplV::getMember(const std::string& name)
    const {
  return members.getField(name).getValue();
}

const std::string& VariantMessage::ValueImplV::getMemberName(size_t index)
    const {
  return members.getField(index).getName();
}

bool VariantMessage::ValueImplV::hasMember(const std::string& name) const {
  return members.hasField(name);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void VariantMessage::Value::writeMember(std::ostream& stream, size_t index)
    const {
  SharedVariant member = getMember(index);
  
  if (!member.getType().isBuiltin()) {
    stream << getMemberName(index) << ":";
  
    std::stringstream memberStream;    
    std::string line;
    
    memberStream << member;

    while (std::getline(memberStream, line))      
      stream << "\n  " << line;
  }
  else
    stream << getMemberName(index) << ": " << member;
}

Variant::ValuePtr VariantMessage::ValueImplV::clone() const {
  return Variant::ValuePtr(new ValueImplV(*this));
}

}
