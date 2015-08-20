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
#include "variant_topic_tools/VariantCollection.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

VariantCollection::VariantCollection() {
}

VariantCollection::VariantCollection(const DataType& type) :
  Variant(type) {
}

VariantCollection::VariantCollection(const VariantCollection& src) :
  Variant(src) {
}

VariantCollection::VariantCollection(const Variant& src) :
  Variant(src) {
  if (value)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Value>(value));
}

VariantCollection::~VariantCollection() {
}

VariantCollection::Value::Value() {
}

VariantCollection::Value::~Value() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t VariantCollection::getNumMembers() const {
  if (value)
    return boost::static_pointer_cast<Value>(value)->getNumMembers();
  else
    return 0;
}

Variant& VariantCollection::getMember(size_t index) {
  if (value)
    return boost::static_pointer_cast<Value>(value)->getMember(index);
  else
    throw NoSuchMemberException(index);
}

const Variant& VariantCollection::getMember(size_t index) const {
  if (value)
    return boost::static_pointer_cast<Value>(value)->getMember(index);
  else
    throw NoSuchMemberException(index);
}

Variant& VariantCollection::getMember(const std::string& name) {
  if (value)
    return boost::static_pointer_cast<Value>(value)->getMember(name, 0);
  else
    throw NoSuchMemberException(name);
}

const Variant& VariantCollection::getMember(const std::string& name) const {
  if (value)
    return boost::static_pointer_cast<Value>(value)->getMember(name, 0);
  else
    throw NoSuchMemberException(name);
}

bool VariantCollection::hasMember(const std::string& name) const {
  if (value)
    return boost::static_pointer_cast<Value>(value)->hasMember(name, 0);
  else
    return false;
}

bool VariantCollection::isEmpty() const {
  if (value)
    return !boost::static_pointer_cast<Value>(value)->getNumMembers();
  else
    return true;
}

Variant& VariantCollection::Value::getMember(const std::string& name, size_t
    pos) {
  pos = name.find_first_not_of('/', pos);
  
  if (pos != std::string::npos) {
    size_t i = name.find_first_of('/', pos);
    
    if (i != std::string::npos) {
      Variant& member = getMember(name.substr(pos, i-pos));
        
      if (member.isCollection())
        return boost::static_pointer_cast<Value>(member.value)->getMember(
          name, i+1);
    }
    else
      return getMember(name.substr(pos));
  }

  throw NoSuchMemberException(name);
}

bool VariantCollection::Value::hasMember(const std::string& name, size_t pos)
    const {
  pos = name.find_first_not_of('/', pos);
  
  if (pos != std::string::npos) {
    size_t i = name.find_first_of('/', pos);
    
    if (i != std::string::npos) {
      const Variant& member = getMember(name.substr(pos, i-pos));
        
      if (member.isCollection())
        return boost::static_pointer_cast<Value>(member.value)->hasMember(
          name, i+1);
    }
    else
      return hasMember(name.substr(pos));
  }

  return false;
}

bool VariantCollection::Value::isEqual(const Variant::Value& value) const {
  if (getNumMembers() == static_cast<const Value&>(value).getNumMembers()) {
    for (size_t i = 0; i < getNumMembers(); ++i)
      if (getMember(i) != static_cast<const Value&>(value).getMember(i))
        return false;
      
    return true;
  }
  else
    return false;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void VariantCollection::Value::read(std::istream& stream) {
  throw InvalidOperationException("Reading a variant collection");
}

void VariantCollection::Value::write(std::ostream& stream) const {
  for (size_t i = 0; i < getNumMembers(); ++i) {
    if (i)
      stream << "\n";
    writeMember(stream, i);
  }  
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

Variant& VariantCollection::operator[](size_t index) {
  return getMember(index);
}

const Variant& VariantCollection::operator[](size_t index) const {
  return getMember(index);
}

Variant& VariantCollection::operator[](const std::string& name) {
  return getMember(name);
}

const Variant& VariantCollection::operator[](const std::string& name) const {
  return getMember(name);
}

}
