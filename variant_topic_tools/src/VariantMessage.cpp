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
    value.reset(new Value(members));
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

VariantMessage::Value::Value(const MessageFieldCollection<Variant>& members) :
  members(members) {
}

VariantMessage::Value::Value(const Value& src) :
  members(src.members) {
}

VariantMessage::Value::~Value() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t VariantMessage::Value::getNumMembers() const {
  return members.getNumFields();
}

Variant& VariantMessage::Value::getMember(size_t index) {
  return members.getField(index).getValue();
}

const Variant& VariantMessage::Value::getMember(size_t index) const {
  return members.getField(index).getValue();
}

Variant& VariantMessage::Value::getMember(const std::string& name) {
  return members.getField(name).getValue();
}

const Variant& VariantMessage::Value::getMember(const std::string& name)
    const {
  return members.getField(name).getValue();
}

bool VariantMessage::Value::hasMember(const std::string& name) const {
  return members.hasField(name);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

Variant::ValuePtr VariantMessage::Value::clone() const {
  return Variant::ValuePtr(new Value(*this));
}

void VariantMessage::Value::writeMember(std::ostream& stream, size_t index)
    const {
  stream << members[index].getName() << ": ";
  
  if (!members[index].getValue().getType().isBuiltin()) {
    std::stringstream memberStream;    
    memberStream << members[index].getValue();
    
    std::string line;
    size_t numLines = 0;
    
    while (std::getline(memberStream, line)) {
      if (!memberStream.eof() || numLines)
        stream << "\n  ";
      
      stream << line;
      ++numLines;
    }
  }
  else    
    stream << members[index].getValue();
}

}
