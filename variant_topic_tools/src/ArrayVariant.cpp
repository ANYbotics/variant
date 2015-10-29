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

#include "variant_topic_tools/ArrayDataType.h"
#include "variant_topic_tools/ArrayVariant.h"
#include "variant_topic_tools/Exceptions.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ArrayVariant::ArrayVariant() {
}

ArrayVariant::ArrayVariant(const DataType& type, const DataType& memberType,
    size_t numMembers) :
  CollectionVariant(type) {
  if (type.isValid())
    value.reset(new ValueImplV(memberType, numMembers));
}

ArrayVariant::ArrayVariant(const ArrayVariant& src) :
  CollectionVariant(src) {
}

ArrayVariant::ArrayVariant(const Variant& src) :
  CollectionVariant(src) {
  if (value)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Value>(value));
}

ArrayVariant::~ArrayVariant() {
}

ArrayVariant::Value::Value() {
}

ArrayVariant::Value::~Value() {
}

ArrayVariant::ValueImplV::ValueImplV(const DataType& memberType, size_t
    numMembers) :
  memberType(memberType),
  numMembers(numMembers),
  members(numMembers) {
  for (size_t i = 0; i < numMembers; ++i)
    members[i] = memberType.createVariant();
}

ArrayVariant::ValueImplV::ValueImplV(const ValueImplV& src) :
  memberType(src.memberType),
  numMembers(src.numMembers),
  members(src.members) {
}

ArrayVariant::ValueImplV::~ValueImplV() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void ArrayVariant::Value::setValue(const Variant::Value& value) {
  const Value& arrayValue = dynamic_cast<const Value&>(value);
  
  resize(arrayValue.getNumMembers());
  
  for (size_t i = 0; i < getNumMembers(); ++i)
    setMember(i, arrayValue.getMember(i));
}

void ArrayVariant::Value::setMember(const std::string& name, const Variant&
    member) {
  size_t index;
  
  try {
    index = boost::lexical_cast<size_t>(name);
  }
  catch (...) {
    throw NoSuchMemberException(name);
  }
    
  return setMember(index, member);
}

Variant ArrayVariant::Value::getMember(const std::string& name) const {
  size_t index;
  
  try {
    index = boost::lexical_cast<size_t>(name);
  }
  catch (...) {
    throw NoSuchMemberException(name);
  }
    
  return getMember(index);
}

bool ArrayVariant::Value::hasMember(const std::string& name) const {
  size_t index;
  
  try {
    index = boost::lexical_cast<size_t>(name);
  }
  catch (...) {
    return false;
  }
  
  return (index < getNumMembers());
}


size_t ArrayVariant::ValueImplV::getNumMembers() const {
  return members.size();
}

void ArrayVariant::ValueImplV::setMember(size_t index, const Variant&
    member) {
  if (index < members.size())
    members[index] = member;
  else
    throw NoSuchMemberException(index);
}

Variant ArrayVariant::ValueImplV::getMember(size_t index) const {
  if (index < members.size())
    return members[index];
  else
    throw NoSuchMemberException(index);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ArrayVariant::addMember(const Variant& member) {
  if (value) {
    if (member.getType().isValid())
      boost::dynamic_pointer_cast<Value>(value)->addMember(member);
    else
      throw InvalidDataTypeException();
  }
  else
    throw InvalidOperationException("Adding a member to an invalid array");
}

void ArrayVariant::resize(size_t numMembers) {
  if (value)
    boost::dynamic_pointer_cast<Value>(value)->resize(numMembers);
  else if (numMembers)
    throw InvalidOperationException("Resizing an invalid array");
}

void ArrayVariant::clear() {
  if (value)
    boost::dynamic_pointer_cast<Value>(value)->clear();
}

void ArrayVariant::Value::writeMember(std::ostream& stream, size_t index)
    const {
  if (!getMember(index).getType().isBuiltin()) {
    stream << boost::lexical_cast<std::string>(index) << ":";
    
    std::stringstream memberStream;
    std::string line;
    
    memberStream << getMember(index);
        
    while (std::getline(memberStream, line))
      stream << "\n  " << line;
  }
  else    
    stream << boost::lexical_cast<std::string>(index) << ": " <<
      getMember(index);
}

void ArrayVariant::ValueImplV::addMember(const Variant& member) {
  if (!numMembers) {
    if (member.getType() == memberType)
      members.push_back(member);
    else
      throw DataTypeMismatchException(memberType.getIdentifier(),
        member.getType().getIdentifier());
  }
  else
    throw InvalidOperationException("Adding a member to a non-dynamic array");
}

void ArrayVariant::ValueImplV::resize(size_t numMembers) {
  if (!this->numMembers || (numMembers == this->numMembers)) {
    if (numMembers != members.size()) {
      size_t i = members.size();
      
      members.resize(numMembers);
      
      for ( ; i < members.size(); ++i)
        members[i] = memberType.createVariant();
    }
  }
  else
    throw InvalidOperationException("Resizing a non-dynamic array");
}

void ArrayVariant::ValueImplV::clear() {
  if (!numMembers)
    members.clear();
  else
    throw InvalidOperationException("Clearing a non-dynamic array");
}

Variant::ValuePtr ArrayVariant::ValueImplV::clone() const {
  return Variant::ValuePtr(new ValueImplV(*this));
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

ArrayVariant& ArrayVariant::operator+=(const Variant& member) {
  addMember(member);
  return *this;
}

}
