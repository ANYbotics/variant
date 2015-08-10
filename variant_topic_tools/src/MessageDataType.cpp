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

#include <boost/regex.hpp>

#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/MessageDataType.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageDataType::MessageDataType() {
}

MessageDataType::MessageDataType(const std::string& identifier, const
    std::vector<MessageMember>& members) {
  impl.reset(new DataType::ImplA(new ImplV(identifier, members)));
}

MessageDataType::MessageDataType(const std::string& identifier, const
    std::string& definition) {
  impl.reset(new DataType::ImplA(new ImplV(identifier, definition)));
}

MessageDataType::MessageDataType(const MessageDataType& src) :
  DataType(src) {
}

MessageDataType::MessageDataType(const DataType& src) :
  DataType(src) {
  if (impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<MessageDataType::Impl>(
      impl->adaptee));
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
  
  const std::string memberTypeExpression("[a-zA-Z][a-zA-Z1-9_/]*");
  const std::string memberNameExpression("[a-zA-Z][a-zA-Z1-9_]*");
  const std::string commentExpression("#.*");
  
  const boost::regex constantMemberExpression(
    "^\\h*("+memberTypeExpression+")\\h+("+memberNameExpression+
    ")\\h*=\\h*(.*)$");
  const boost::regex variableMemberExpression(
    "^\\h*("+memberTypeExpression+")\\h+("+memberNameExpression+
    ")("+commentExpression+")?$");
  
  std::istringstream stream(definition);
  std::string line;
  
  while (std::getline(stream, line)) {
    boost::smatch match;
    
    if (boost::regex_match(line, match, variableMemberExpression)) {
      std::string memberName(match[2].first, match[2].second);
      std::string memberType(match[1].first, match[1].second);
      
      MessageVariable member(memberName, memberType);
      members.push_back(member);
    }
    else if (boost::regex_match(line, match, constantMemberExpression)) {
      std::string memberName(match[2].first, match[2].second);
      std::string memberType(match[1].first, match[1].second);
      std::string memberValue(match[3].first, match[3].second);
      
      MessageConstant member(memberName, memberType, memberValue);
      members.push_back(member);
    }
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
    return boost::dynamic_pointer_cast<Impl>(impl->adaptee)->getMD5Sum();
}

const std::string& MessageDataType::getDefinition() const {
  if (!impl) {
    static std::string definition;
    return definition;
  }
  else
    return boost::dynamic_pointer_cast<Impl>(impl->adaptee)->getDefinition();
}

size_t MessageDataType::getNumMembers() const {
  if (impl)
    return boost::dynamic_pointer_cast<Impl>(impl->adaptee)->members.size();
  else
    return 0;
}

const MessageMember& MessageDataType::getMember(size_t index) const {
  if (index < getNumMembers())
    return boost::dynamic_pointer_cast<Impl>(impl->adaptee)->members[index];
  else
    throw NoSuchMessageMemberException(index);
}

bool MessageDataType::isSimple() const {
  if (impl)
    return boost::dynamic_pointer_cast<Impl>(impl->adaptee)->isSimple();
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
  MessageVariable variable(name, type);
  addMember(variable);
  
  return variable;
}

void MessageDataType::addMember(const MessageMember& member) {
  if (impl)
    boost::dynamic_pointer_cast<Impl>(impl->adaptee)->addMember(member);
  else
    throw InvalidDataTypeException();
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
    BOOST_ASSERT(boost::dynamic_pointer_cast<MessageDataType::Impl>(
      impl->adaptee));
    
  return *this;
}

const MessageMember& MessageDataType::operator[](size_t index) const {
  return getMember(index);
}

}
