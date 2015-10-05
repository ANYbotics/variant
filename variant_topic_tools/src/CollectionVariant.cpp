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

#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/CollectionVariant.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CollectionVariant::CollectionVariant() {
}

CollectionVariant::CollectionVariant(const DataType& type) :
  Variant(type) {
}

CollectionVariant::CollectionVariant(const CollectionVariant& src) :
  Variant(src) {
}

CollectionVariant::CollectionVariant(const Variant& src) :
  Variant(src) {
  if (value)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Value>(value));
}

CollectionVariant::~CollectionVariant() {
}

CollectionVariant::Value::Value() {
}

CollectionVariant::Value::~Value() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t CollectionVariant::getNumMembers() const {
  if (value)
    return boost::dynamic_pointer_cast<Value>(value)->getNumMembers();
  else
    return 0;
}

void CollectionVariant::setMember(size_t index, const Variant& member) {
  if (value)
    boost::dynamic_pointer_cast<Value>(value)->setMember(index, member);
  else
    throw NoSuchMemberException(index);
}

void CollectionVariant::setMember(const std::string& name, const Variant&
    member) {
  if (value)
    boost::dynamic_pointer_cast<Value>(value)->setMember(name, member, 0);
  else
    throw NoSuchMemberException(name);
}

SharedVariant CollectionVariant::getMember(size_t index) const {
  if (value)
    return boost::dynamic_pointer_cast<Value>(value)->getMember(index);
  else
    throw NoSuchMemberException(index);
}

SharedVariant CollectionVariant::getMember(const std::string& name) const {
  if (value)
    return boost::dynamic_pointer_cast<Value>(value)->getMember(name, 0);
  else
    throw NoSuchMemberException(name);
}

bool CollectionVariant::hasMember(const std::string& name) const {
  if (value)
    return boost::dynamic_pointer_cast<Value>(value)->hasMember(name, 0);
  else
    return false;
}

bool CollectionVariant::isEmpty() const {
  if (value)
    return !boost::dynamic_pointer_cast<Value>(value)->getNumMembers();
  else
    return true;
}

void CollectionVariant::Value::setMember(const std::string& name, const
    Variant& member, size_t pos) {
  pos = name.find_first_not_of('/', pos);
  
  if (pos != std::string::npos) {
    size_t i = name.find_first_of('/', pos);
    
    if (i != std::string::npos) {
      SharedVariant sharedMember = getMember(name.substr(pos, i-pos));
        
      if (sharedMember.isCollection()) {
        boost::dynamic_pointer_cast<Value>(sharedMember.value)->setMember(
          name, member, i+1);
        return;
      }
    }
    else {
      setMember(name.substr(pos), member);
      return;
    }
  }

  throw NoSuchMemberException(name);
}

SharedVariant CollectionVariant::Value::getMember(const std::string& name,
    size_t pos) const {
  pos = name.find_first_not_of('/', pos);
  
  if (pos != std::string::npos) {
    size_t i = name.find_first_of('/', pos);
    
    if (i != std::string::npos) {
      SharedVariant sharedMember = getMember(name.substr(pos, i-pos));
        
      if (sharedMember.isCollection())
        return boost::dynamic_pointer_cast<Value>(sharedMember.value)->
          getMember(name, i+1);
    }
    else
      return getMember(name.substr(pos));
  }

  throw NoSuchMemberException(name);
}

bool CollectionVariant::Value::hasMember(const std::string& name, size_t pos)
    const {
  pos = name.find_first_not_of('/', pos);
  
  if (pos != std::string::npos) {
    size_t i = name.find_first_of('/', pos);
    
    if (i != std::string::npos) {
      SharedVariant sharedMember = getMember(name.substr(pos, i-pos));
      
      if (sharedMember.isCollection())
        return boost::dynamic_pointer_cast<Value>(sharedMember.value)->
          hasMember(name, i+1);
    }
    else
      return hasMember(name.substr(pos));
  }

  return false;
}

bool CollectionVariant::Value::isEqual(const Variant::Value& value) const {
  const Value& collectionValue = dynamic_cast<const Value&>(value);
  
  if (getNumMembers() == collectionValue.getNumMembers()) {
    for (size_t i = 0; i < getNumMembers(); ++i)
      if (getMember(i) != collectionValue.getMember(i))
        return false;
      
    return true;
  }
  else
    return false;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CollectionVariant::Value::read(std::istream& stream) {
  throw InvalidOperationException("Reading a collection variant");
}

void CollectionVariant::Value::write(std::ostream& stream) const {
  for (size_t i = 0; i < getNumMembers(); ++i) {
    if (i)
      stream << "\n";
    writeMember(stream, i);
  }  
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

SharedVariant CollectionVariant::operator[](size_t index) const {
  return getMember(index);
}

SharedVariant CollectionVariant::operator[](const std::string& name) const {
  return getMember(name);
}

}
