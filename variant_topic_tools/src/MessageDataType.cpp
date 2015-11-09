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
#include "variant_topic_tools/MessageTypeParser.h"
#include "variant_topic_tools/MessageVariable.h"
#include "variant_topic_tools/MessageVariant.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageDataType::MessageDataType() {
}

MessageDataType::MessageDataType(const std::string& identifier,
    const MessageFieldCollection<MessageConstant>& constantMembers,
    const MessageFieldCollection<MessageVariable>& variableMembers) {
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

MessageDataType::Impl::Impl(const MessageFieldCollection<MessageConstant>&
    constantMembers, const MessageFieldCollection<MessageVariable>&
    variableMembers) :
  constantMembers(constantMembers),
  variableMembers(variableMembers) {
  for (size_t i = 0; i < constantMembers.getNumFields(); ++i)
    if (!constantMembers[i].isValid())
      throw InvalidMessageMemberException();
    
  for (size_t i = 0; i < variableMembers.getNumFields(); ++i)
    if (!variableMembers[i].isValid())
      throw InvalidMessageMemberException();
}

MessageDataType::Impl::Impl(const std::string& identifier, const std::string&
    definition) {
  BOOST_ASSERT(!definition.empty());
  
  std::string package, plainType;
  if (!MessageTypeParser::matchType(identifier, package, plainType))
    throw InvalidMessageTypeException(identifier);
    
  DataTypeRegistry registry;
  std::istringstream stream(definition);
  std::string line;
  
  while (std::getline(stream, line)) {
    std::string memberName, memberType, memberValue;
    
    if (MessageDefinitionParser::matchVariable(line, memberName,
        memberType)) {
      std::string bareMemberType, memberPackage, plainMemberType;
      bool isArrayMember = false;
      size_t memberSize;
    
      if (MessageDefinitionParser::matchArray(line, memberName,
          bareMemberType, memberSize))
        isArrayMember = true;
      else
        bareMemberType = memberType;
      
      if (!MessageTypeParser::matchType(bareMemberType, memberPackage,
          plainMemberType))
        throw InvalidMessageTypeException(bareMemberType);
    
      if (!registry.getDataType(bareMemberType).isBuiltin()) {
        if (memberPackage.empty()) {
          if (plainMemberType == "Header")
            memberPackage = "std_msgs";
          else
            memberPackage = package;
          
          if (isArrayMember) {
            if (memberSize)
              memberType = memberPackage+"/"+plainMemberType+"["+
                boost::lexical_cast<std::string>(memberSize)+"]";
            else
              memberType = memberPackage+"/"+plainMemberType+"[]";
          }
          else
            memberType = memberPackage+"/"+plainMemberType;
        }
      }
      
      if (registry.getDataType(memberType).isValid()) {
        MessageVariable member(memberName, memberType);
        variableMembers.appendField(memberName, member);
      }
      else
        throw NoSuchDataTypeException(memberType);
    }
    else if (MessageDefinitionParser::matchConstant(line, memberName,
        memberType, memberValue)) {
      if (registry.getDataType(memberType).isValid()) {
        MessageConstant member(memberName, memberType, memberValue);
        constantMembers.appendField(memberName, member);
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
    const MessageFieldCollection<MessageConstant>& constantMembers,
    const MessageFieldCollection<MessageVariable>& variableMembers) :
  Impl(constantMembers, variableMembers),
  identifier(identifier) {  
  std::ostringstream stream;
  
  for (size_t i = 0; i < constantMembers.getNumFields(); ++i)
    stream << constantMembers[i] << "\n";
  
  for (size_t i = 0; i < variableMembers.getNumFields(); ++i)
    stream << variableMembers[i] << "\n";
  
  definition = stream.str();
  
  recalculateMD5Sum();
}

MessageDataType::ImplV::ImplV(const std::string& identifier, const
    std::string& definition) :
  Impl(identifier, definition),
  identifier(identifier),
  definition(definition) {  
  recalculateMD5Sum();
}

MessageDataType::ImplV::~ImplV() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string MessageDataType::getMD5Sum() const {
  if (impl)
    return boost::static_pointer_cast<Impl>(*impl)->getMD5Sum();
  else
    return std::string("*");
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
    return boost::static_pointer_cast<Impl>(*impl)->constantMembers.
      getNumFields();
  else
    return 0;
}

size_t MessageDataType::getNumVariableMembers() const {
  if (impl)
    return boost::static_pointer_cast<Impl>(*impl)->variableMembers.
      getNumFields();
  else
    return 0;
}

const MessageMember& MessageDataType::getMember(size_t index) const {
  if (index < getNumConstantMembers())
    return boost::static_pointer_cast<Impl>(*impl)->constantMembers[index].
      getValue();
  else if (index < getNumConstantMembers()+getNumVariableMembers())
    return boost::static_pointer_cast<Impl>(*impl)->variableMembers[
      index-getNumConstantMembers()].getValue();
  else
    throw NoSuchMemberException(index);
}

const MessageConstant& MessageDataType::getConstantMember(size_t index) const {
  if (index < getNumConstantMembers())
    return boost::static_pointer_cast<Impl>(*impl)->constantMembers[index].
      getValue();
  else
    throw NoSuchMemberException(index);
}

const MessageVariable& MessageDataType::getVariableMember(size_t index) const {
  if (index < getNumVariableMembers())
    return boost::static_pointer_cast<Impl>(*impl)->variableMembers[index].
      getValue();
  else
    throw NoSuchMemberException(index);
}

bool MessageDataType::hasHeader() const {
  if (impl)
    return (boost::static_pointer_cast<Impl>(*impl)->variableMembers.
        hasField("header") &&
      (boost::static_pointer_cast<Impl>(*impl)->variableMembers["header"].
        getValue().getType().getIdentifier() == "std_msgs/Header"));
  else
    return false;
}

const std::string& MessageDataType::ImplV::getIdentifier() const {
  return identifier;
}

std::string MessageDataType::ImplV::getMD5Sum() const {
  return md5Sum.toString();
}

const std::string& MessageDataType::ImplV::getDefinition() const {
  return definition;
}

size_t MessageDataType::ImplV::getSize() const {
  if (isFixedSize()) {
    size_t size = 0;
    
    for (size_t i = 0; i < variableMembers.getNumFields(); ++i)
      size += variableMembers[i].getValue().getType().getSize();
  
    return size;
  }
  else
    return 0;
}

bool MessageDataType::ImplV::isFixedSize() const {
  bool fixedSize = true;
  
  for (size_t i = 0; i < variableMembers.getNumFields(); ++i)
    fixedSize &= variableMembers[i].getValue().getType().isFixedSize();
  
  return fixedSize;
}

bool MessageDataType::ImplV::isSimple() const {
  return false;
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
  MessageFieldCollection<Serializer> memberSerializers;
  
  for (size_t i = 0; i < variableMembers.getNumFields(); ++i)
    memberSerializers.appendField(variableMembers[i].getName(),
      variableMembers[i].getValue().getType().createSerializer());
  
  return MessageSerializer(memberSerializers);
}

Variant MessageDataType::ImplV::createVariant(const DataType& type) const {
  MessageFieldCollection<Variant> members;
  
  for (size_t i = 0; i < variableMembers.getNumFields(); ++i)
    members.appendField(variableMembers[i].getName(),
      variableMembers[i].getValue().getType().createVariant());
  
  return MessageVariant(type, members);
}
  
void MessageDataType::ImplV::addConstantMember(const MessageConstant&
    member) {
  constantMembers.appendField(member.getName(), member);

  std::ostringstream stream;
  stream << member << "\n";

  definition += stream.str();
  
  recalculateMD5Sum();
}

void MessageDataType::ImplV::addVariableMember(const MessageVariable&
    member) {
  variableMembers.appendField(member.getName(), member);

  std::ostringstream stream;
  stream << member << "\n";

  definition += stream.str();
  
  recalculateMD5Sum();
}

void MessageDataType::ImplV::recalculateMD5Sum() {
  std::ostringstream stream;
  
  for (size_t i = 0; i < constantMembers.getNumFields(); ++i) {
    const MessageConstant& constantMember = constantMembers[i].getValue();
    
    stream << constantMember.getType().getIdentifier() << " " <<
      constantMember.getName() << "=" << constantMember.getValue() << "\n";
  }
  
  for (size_t i = 0; i < variableMembers.getNumFields(); ++i) {
    const MessageVariable& variableMember = variableMembers[i].getValue();
    
    DataType memberType = variableMember.getType();
    DataType bareMemberType = memberType;
    
    while (bareMemberType.isArray()) {
      ArrayDataType arrayMemberType = bareMemberType;
      bareMemberType = arrayMemberType.getMemberType();
    }
    
    if (bareMemberType.isBuiltin()) {
      stream << memberType.getIdentifier() << " " <<
        variableMember.getName() << "\n";
    }
    else if (bareMemberType.isMessage()) {
      MessageDataType messageMemberType = bareMemberType;
      stream << messageMemberType.getMD5Sum() << " " <<
        variableMember.getName() << "\n";
    }
  }
  
  std::string md5SumText = stream.str();
  if (!md5SumText.empty())
    md5SumText.erase(md5SumText.size()-1);
  
  md5Sum.clear();  
  md5Sum.update(md5SumText);
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
