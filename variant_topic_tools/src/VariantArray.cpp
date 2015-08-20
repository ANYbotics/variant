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
#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/VariantArray.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

VariantArray::VariantArray() {
}

VariantArray::VariantArray(const ArrayDataType& type, size_t numMembers) :
  VariantCollection(static_cast<const DataType&>(type)) {
  if (type.isValid())
    value.reset(new ValueV(type.getElementType(), numMembers));
}

VariantArray::VariantArray(const VariantArray& src) :
  VariantCollection(src) {
}

VariantArray::VariantArray(const Variant& src) :
  VariantCollection(src) {
  if (value)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Value>(value));
}

VariantArray::~VariantArray() {
}

VariantArray::Value::Value() {
}

VariantArray::Value::~Value() {
}

VariantArray::ValueV::ValueV(const DataType& memberType, size_t numMembers) :
  memberType(memberType),
  numMembers(numMembers),
  members(numMembers, memberType.createVariant()) {
}

VariantArray::ValueV::ValueV(const ValueV& src) :
  memberType(src.memberType),
  numMembers(src.numMembers),
  members(src.members) {
}

VariantArray::ValueV::~ValueV() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

bool VariantArray::isFixedSize() const {
  if (value)
    return boost::static_pointer_cast<Value>(value)->isFixedSize();
  else
    return true;
}

Variant& VariantArray::Value::getMember(const std::string& name) {
  size_t index;
  
  try {
    index = boost::lexical_cast<size_t>(name);
  }
  catch (...) {
    throw NoSuchMemberException(name);
  }
    
  return getMember(index);
}

const Variant& VariantArray::Value::getMember(const std::string& name)
    const {
  size_t index;
  
  try {
    index = boost::lexical_cast<size_t>(name);
  }
  catch (...) {
    throw NoSuchMemberException(name);
  }
    
  return getMember(index);
}

bool VariantArray::Value::hasMember(const std::string& name) const {
  size_t index;
  
  try {
    index = boost::lexical_cast<size_t>(name);
  }
  catch (...) {
    return false;
  }
  
  return (index < getNumMembers());
}

size_t VariantArray::ValueV::getNumMembers() const {
  return members.size();
}

Variant& VariantArray::ValueV::getMember(size_t index) {
  if (index < members.size())
    return members[index];
  else
    throw NoSuchMemberException(index);
}

const Variant& VariantArray::ValueV::getMember(size_t index) const {
  if (index < members.size())
    return members[index];
  else
    throw NoSuchMemberException(index);
}

bool VariantArray::ValueV::isFixedSize() const {
  return numMembers;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void VariantArray::addMember(const Variant& member) {
  if (value) {
    if (member.getType().isValid())
      boost::static_pointer_cast<Value>(value)->addMember(member);
    else
      throw InvalidDataTypeException();
  }
  else
    throw InvalidOperationException("Adding a member to an invalid array");
}

void VariantArray::resize(size_t numMembers) {
  if (value)
    boost::static_pointer_cast<Value>(value)->resize(numMembers);
  else if (numMembers)
    throw InvalidOperationException("Resizing an invalid array");
}

void VariantArray::clear() {
  if (value)
    boost::static_pointer_cast<Value>(value)->clear();
}

void VariantArray::Value::writeMember(std::ostream& stream, size_t index)
    const {
  stream << boost::lexical_cast<std::string>(index) << ": ";
  
  if (!getMember(index).getType().isBuiltin()) {
    std::stringstream memberStream;
    memberStream << getMember(index);
    
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
    stream << getMember(index);
}

void VariantArray::ValueV::addMember(const Variant& member) {
  if (!numMembers) {
    if (member.getType() == memberType)
      members.push_back(member);
    else
      throw DataTypeMismatchException(memberType.getIdentifier(),
        member.getType().getIdentifier());
  }
  else
    throw InvalidOperationException("Adding a member to a fixed-size array");
}

void VariantArray::ValueV::resize(size_t numMembers) {
  if (!this->numMembers || (numMembers == this->numMembers)) {
    if (numMembers != members.size())
      members.resize(numMembers, memberType.createVariant());
  }
  else
    throw InvalidOperationException("Resizing a fixed-size array");
}

void VariantArray::ValueV::clear() {
  if (!numMembers)
    members.clear();
  else
    throw InvalidOperationException("Clearing a fixed-size array");
}

Variant::ValuePtr VariantArray::ValueV::clone() const {
  return Variant::ValuePtr(new ValueV(*this));
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

VariantArray& VariantArray::operator+=(const Variant& member) {
  addMember(member);
  return *this;
}

}
