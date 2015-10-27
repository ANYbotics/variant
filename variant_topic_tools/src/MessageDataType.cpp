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
#include "variant_topic_tools/MessageConstant.h"
#include "variant_topic_tools/MessageDataType.h"
#include "variant_topic_tools/MessageDefinitionParser.h"
#include "variant_topic_tools/MessageMember.h"
#include "variant_topic_tools/MessageVariable.h"
#include "variant_topic_tools/MessageVariant.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageDataType::MessageDataType() {
}

MessageDataType::MessageDataType(const std::string& identifier,
    const std::vector<MessageConstant>& constantMembers,
    const std::vector<MessageVariable>& variableMembers) {
  impl.reset(new boost::shared_ptr<DataType::Impl>(
    new ImplV(identifier, constantMembers, variableMembers)));
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

MessageDataType::Impl::Impl(const std::vector<MessageConstant>&
    constantMembers, const std::vector<MessageVariable>& variableMembers) :
  constantMembers(constantMembers),
  variableMembers(variableMembers) {
  for (size_t i = 0; i < constantMembers.size(); ++i)
    if (!constantMembers[i].isValid())
      throw InvalidMessageMemberException();
    
  for (size_t i = 0; i < variableMembers.size(); ++i)
    if (!variableMembers[i].isValid())
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
        MessageVariable member(memberName, memberType);
        variableMembers.push_back(member);
      }
      else
        throw NoSuchDataTypeException(memberType);
    }
    else if (MessageDefinitionParser::matchConstant(line, memberName,
        memberType, memberValue)) {
      if (registry.getDataType(memberType).isValid()) {
        MessageConstant member(memberName, memberType, memberValue);
        constantMembers.push_back(member);
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

MessageDataType::ImplV::ImplV(const std::string& identifier,
    const std::vector<MessageConstant>& constantMembers,
    const std::vector<MessageVariable>& variableMembers) :
  Impl(constantMembers, variableMembers),
  identifier(identifier) {  
  std::ostringstream stream;
  
  for (size_t i = 0; i < constantMembers.size(); ++i)
    stream << constantMembers[i] << "\n";
  
  for (size_t i = 0; i < variableMembers.size(); ++i)
    stream << variableMembers[i] << "\n";
  
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
  return getNumConstantMembers()+getNumVariableMembers();
}

size_t MessageDataType::getNumConstantMembers() const {
  if (impl)
    return boost::static_pointer_cast<Impl>(*impl)->constantMembers.size();
  else
    return 0;
}

size_t MessageDataType::getNumVariableMembers() const {
  if (impl)
    return boost::static_pointer_cast<Impl>(*impl)->variableMembers.size();
  else
    return 0;
}

const MessageMember& MessageDataType::getMember(size_t index) const {
  if (index < getNumConstantMembers())
    return boost::static_pointer_cast<Impl>(*impl)->constantMembers[index];
  else if (index < getNumConstantMembers()+getNumVariableMembers())
    return boost::static_pointer_cast<Impl>(*impl)->variableMembers[
      index-getNumConstantMembers()];
  else
    throw NoSuchMemberException(index);
}

const MessageConstant& MessageDataType::getConstantMember(size_t index) const {
  if (index < getNumConstantMembers())
    return boost::static_pointer_cast<Impl>(*impl)->constantMembers[index];
  else
    throw NoSuchMemberException(index);
}

const MessageVariable& MessageDataType::getVariableMember(size_t index) const {
  if (index < getNumVariableMembers())
    return boost::static_pointer_cast<Impl>(*impl)->variableMembers[index];
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
    
    for (size_t i = 0; i < variableMembers.size(); ++i)
      size += variableMembers[i].getSize();
  
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
  
  for (size_t i = 0; i < variableMembers.size(); ++i)
    fixedSize &= variableMembers[i].isFixedSize();
  
  return fixedSize;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageDataType::addConstantMember(const MessageConstant& member) {
  if (impl)
    boost::static_pointer_cast<Impl>(*impl)->addConstantMember(member);
  else
    throw InvalidDataTypeException();
}

MessageConstant MessageDataType::addConstantMember(const std::string& name,
    const Variant& value) {
  MessageConstant member(name, value);
  addConstantMember(member);
  
  return member;
}

void MessageDataType::addVariableMember(const MessageVariable& member) {
  if (impl)
    boost::static_pointer_cast<Impl>(*impl)->addVariableMember(member);
  else
    throw InvalidDataTypeException();
}

MessageVariable MessageDataType::addVariableMember(const std::string& name,
    const DataType& type) {
  MessageVariable member(name, type);
  addVariableMember(member);
  
  return member;
}

Serializer MessageDataType::ImplV::createSerializer(const DataType& type)
    const {
  std::vector<Serializer> memberSerializers;
  memberSerializers.reserve(variableMembers.size());
  
  for (size_t i = 0; i < variableMembers.size(); ++i) {
    memberSerializers.push_back(variableMembers[i].getType().
      createSerializer());
  }
  
  return MessageSerializer(memberSerializers);
}

Variant MessageDataType::ImplV::createVariant(const DataType& type) const {
  return MessageVariant(type, variableMembers);
}
  
void MessageDataType::ImplV::addConstantMember(const MessageConstant&
    member) {
  constantMembers.push_back(member);

  std::ostringstream stream;
  stream << member << "\n";

  definition += stream.str();
}

void MessageDataType::ImplV::addVariableMember(const MessageVariable&
    member) {
  variableMembers.push_back(member);

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
