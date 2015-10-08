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

#include <sstream>

#include "variant_topic_tools/DataTypeRegistry.h"
#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/MessageDataType.h"
#include "variant_topic_tools/MessageDefinitionParser.h"
#include "variant_topic_tools/MessageVariant.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageDataType::MessageDataType() {
}

MessageDataType::MessageDataType(const std::string& identifier, const
    std::vector<MessageMember>& members) {
  impl.reset(new boost::shared_ptr<DataType::Impl>(
    new ImplV(identifier, members)));
}

MessageDataType::MessageDataType(const std::string& identifier, const
    std::string& definition) {
  impl.reset(new boost::shared_ptr<DataType::Impl>(
    new ImplV(identifier, definition)));
}

MessageDataType::MessageDataType(const MessageDataType& src) :
  DataType(src) {
}

MessageDataType::MessageDataType(const DataType& src) :
  DataType(src) {
  if (impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(*impl));
}

MessageDataType::~MessageDataType() {
}

MessageDataType::Impl::Impl(const std::vector<MessageMember>& members) :
  members(members) {
  for (size_t i = 0; i < members.size(); ++i)
    if (!members[i].isValid())
      throw InvalidMessageMemberException();
}

MessageDataType::Impl::Impl(const std::string& definition) {
  BOOST_ASSERT(!definition.empty());
  
  DataTypeRegistry registry;
  std::istringstream stream(definition);
  std::string line;
  
  while (std::getline(stream, line)) {
    std::string memberName, memberType, memberValue;
    
    if (MessageDefinitionParser::matchVariable(line, memberName,
        memberType)) {
      if (memberType == "Header")
        memberType = "std_msgs/Header";
      
      if (registry.getDataType(memberType).isValid()) {
        MessageVariable member(memberName, memberType, 0);
        members.push_back(member);
      }
      else
        throw NoSuchDataTypeException(memberType);
    }
    else if (MessageDefinitionParser::matchConstant(line, memberName,
        memberType, memberValue)) {
      if (registry.getDataType(memberType).isValid()) {
        MessageConstant member(memberName, memberType, memberValue);
        members.push_back(member);
      }
      else
        throw NoSuchDataTypeException(memberType);
    }
    else if (MessageDefinitionParser::matchSeparator(line))
      break;
  }
}

MessageDataType::Impl::~Impl() {
}

MessageDataType::ImplV::ImplV(const std::string& identifier, const
    std::vector<MessageMember>& members) :
  Impl(members),
  identifier(identifier) {  
  std::ostringstream stream;
  for (size_t i = 0; i < members.size(); ++i)
    stream << members[i] << "\n";
  
  definition = stream.str();
}

MessageDataType::ImplV::ImplV(const std::string& identifier, const
    std::string& definition) :
  Impl(definition),
  identifier(identifier),
  definition(definition) {
}

MessageDataType::ImplV::~ImplV() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const std::string& MessageDataType::getMD5Sum() const {
  if (!impl) {
    static std::string md5Sum;
    return md5Sum;
  }
  else
    return boost::static_pointer_cast<Impl>(*impl)->getMD5Sum();
}

const std::string& MessageDataType::getDefinition() const {
  if (!impl) {
    static std::string definition;
    return definition;
  }
  else
    return boost::static_pointer_cast<Impl>(*impl)->getDefinition();
}

size_t MessageDataType::getNumMembers() const {
  if (impl)
    return boost::static_pointer_cast<Impl>(*impl)->members.size();
  else
    return 0;
}

const MessageMember& MessageDataType::getMember(size_t index) const {
  if (index < getNumMembers())
    return boost::static_pointer_cast<Impl>(*impl)->members[index];
  else
    throw NoSuchMemberException(index);
}

bool MessageDataType::isSimple() const {
  if (impl)
    return boost::static_pointer_cast<Impl>(*impl)->isSimple();
  else
    return false;
}

const std::string& MessageDataType::ImplV::getIdentifier() const {
  return identifier;
}

size_t MessageDataType::ImplV::getSize() const {
  if (isFixedSize()) {
    size_t size = 0;
    
    for (size_t i = 0; i < members.size(); ++i)
      size += members[i].getSize();
  
    return size;
  }
  else
    return 0;
}

const std::string& MessageDataType::ImplV::getMD5Sum() const {
  static std::string md5Sum("*");
  return md5Sum;
}

const std::string& MessageDataType::ImplV::getDefinition() const {
  return definition;
}

bool MessageDataType::ImplV::isSimple() const {
  return false;
}

bool MessageDataType::ImplV::isFixedSize() const {
  bool fixedSize = true;
  
  for (size_t i = 0; i < members.size(); ++i)
    fixedSize &= members[i].isFixedSize();
  
  return fixedSize;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

MessageConstant MessageDataType::addConstant(const std::string& name, const
    Variant& value) {
  MessageConstant constant(name, value);
  addMember(constant);
  
  return constant;
}

MessageVariable MessageDataType::addVariable(const std::string& name, const
    DataType& type) {
  MessageVariable variable(name, type, 0);
  addMember(variable);
  
  return variable;
}

void MessageDataType::addMember(const MessageMember& member) {
  if (impl)
    boost::static_pointer_cast<Impl>(*impl)->addMember(member);
  else
    throw InvalidDataTypeException();
}

Serializer MessageDataType::ImplV::createSerializer(const DataType& type)
    const {
  std::vector<Serializer> memberSerializers;
  memberSerializers.reserve(members.size());
  
  for (size_t i = 0; i < members.size(); ++i)
    memberSerializers.push_back(members[i].getType().createSerializer());
  
  return MessageSerializer(memberSerializers);
}

Variant MessageDataType::ImplV::createVariant(const DataType& type) const {
  return MessageVariant(type, members);
}
  
void MessageDataType::ImplV::addMember(const MessageMember& member) {
  members.push_back(member);

  std::ostringstream stream;
  stream << member << "\n";

  definition += stream.str();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

MessageDataType& MessageDataType::operator=(const DataType& src) {
  DataType::operator=(src);
  
  if (impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<MessageDataType::Impl>(*impl));
    
  return *this;
}

const MessageMember& MessageDataType::operator[](size_t index) const {
  return getMember(index);
}

}
