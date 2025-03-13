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

#include "variant_topic_tools/MessageVariant.h"
#include <variant_topic_tools/MessageSerializer.h>
#include <variant_topic_tools/MessageVariable.h>
#include "variant_topic_tools/MessageDataType.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageVariant::MessageVariant() = default;

MessageVariant::MessageVariant(const DataType& type, const MessageFieldCollection<Variant>& members) : CollectionVariant(type) {
  if (type.isValid()) {
    value.reset(new ValueImplV(members));
  }
}

MessageVariant::MessageVariant(const MessageVariant& src) = default;

MessageVariant::MessageVariant(const Variant& src) : CollectionVariant(src) {
  if (value) {
    BOOST_ASSERT(boost::dynamic_pointer_cast<Value>(value));
  }
}

MessageVariant::~MessageVariant() = default;

MessageVariant::Value::Value() = default;

MessageVariant::Value::~Value() = default;

MessageVariant::ValueImplV::ValueImplV(const MessageFieldCollection<Variant>& members) : members(members) {}

MessageVariant::ValueImplV::ValueImplV(const ValueImplV& src) : members(src.members) {}

MessageVariant::ValueImplV::~ValueImplV() = default;

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageVariant::Value::setValue(const Variant::Value& value) {
  const auto& messageValue = dynamic_cast<const Value&>(value);

  for (size_t i = 0; i < getNumMembers(); ++i) {
    setMember(i, messageValue.getMember(i));
  }
}

size_t MessageVariant::ValueImplV::getNumMembers() const {
  return members.getNumFields();
}

void MessageVariant::ValueImplV::setMember(int index, const Variant& member) {
  members.getField(index).setValue(member);
}

void MessageVariant::ValueImplV::setMember(const std::string& name, const Variant& member) {
  members.getField(name).setValue(member);
}

Variant MessageVariant::ValueImplV::getMember(int index) const {
  return members.getField(index).getValue();
}

Variant MessageVariant::ValueImplV::getMember(const std::string& name) const {
  return members.getField(name).getValue();
}

const std::string& MessageVariant::ValueImplV::getMemberName(int index) const {
  return members.getField(index).getName();
}

bool MessageVariant::ValueImplV::hasMember(const std::string& name) const {
  return members.hasField(name);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageVariant::Value::writeMember(std::ostream& stream, int index) const {
  Variant member = getMember(index);

  if (!member.getType().isBuiltin()) {
    stream << getMemberName(index) << ":";

    std::stringstream memberStream;
    std::string line;

    memberStream << member;

    while (std::getline(memberStream, line)) {
      stream << "\n  " << line;
    }
  } else {
    stream << getMemberName(index) << ": " << member;
  }
}

Variant::ValuePtr MessageVariant::ValueImplV::clone() const {
  return Variant::ValuePtr(new ValueImplV(*this));
}

Serializer MessageVariant::ValueImplV::createSerializer(const DataType& /*type*/) const {
  MessageFieldCollection<Serializer> memberSerializers;

  for (size_t i = 0; i < members.getNumFields(); ++i) {
    memberSerializers.appendField(members[i].getName(), members[i].getValue().createSerializer());
  }

  return MessageSerializer(memberSerializers);
}

}  // namespace variant_topic_tools
