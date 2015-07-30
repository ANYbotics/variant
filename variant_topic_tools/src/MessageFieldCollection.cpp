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

#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/MessageField.h"
#include "variant_topic_tools/MessageFieldCollection.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageFieldCollection::MessageFieldCollection() {
}

MessageFieldCollection::MessageFieldCollection(const MessageFieldCollection&
    src) :
  fieldsInOrder(src.fieldsInOrder),
  fieldsByName(src.fieldsByName) {
}

MessageFieldCollection::~MessageFieldCollection() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t MessageFieldCollection::getNumFields() const {
  return fieldsInOrder.size();
}

MessageField& MessageFieldCollection::getField(size_t index) {
  if (index < fieldsInOrder.size())
    return *(fieldsInOrder[index]);
  else
    throw BadFieldIndexException(index);
}

const MessageField& MessageFieldCollection::getField(size_t index) const {
  if (index < fieldsInOrder.size())
    return *(fieldsInOrder[index]);
  else
    throw BadFieldIndexException(index);
}

MessageField& MessageFieldCollection::getField(const std::string& name) {
  return getField(name, 0);
}

const MessageField& MessageFieldCollection::getField(const std::string& name)
    const {
  return getField(name, 0);
}

MessageField& MessageFieldCollection::getField(const std::string& name,
    size_t pos) const {
  pos = name.find_first_not_of('/', pos);
  
  if (pos != std::string::npos) {
    size_t i = name.find_last_not_of('/', pos);
    
    if (i != std::string::npos) {
      std::map<std::string, MessageFieldPtr>::const_iterator it =
        fieldsByName.find(name.substr(pos, i-pos+1));
        
      if (it != fieldsByName.end())
        return it->second->getField(name, pos+1);
    }
    else {
      std::map<std::string, MessageFieldPtr>::const_iterator it =
        fieldsByName.find(name.substr(pos));
      
      if (it != fieldsByName.end())
        return *(it->second);
    }      
  }

  throw BadFieldNameException(name);;
}

bool MessageFieldCollection::hasField(const std::string& name) const {
  return hasField(name, 0);
}

bool MessageFieldCollection::hasField(const std::string& name, size_t pos)
    const {
  pos = name.find_first_not_of('/', pos);
  
  if (pos != std::string::npos) {
    size_t i = name.find_last_not_of('/', pos);
    
    if (i != std::string::npos) {
      std::map<std::string, MessageFieldPtr>::const_iterator it =
        fieldsByName.find(name.substr(pos, i-pos+1));
        
      return ((it != fieldsByName.end()) &&
        it->second->hasField(name, pos+1));
    }
    else {
      std::map<std::string, MessageFieldPtr>::const_iterator it =
        fieldsByName.find(name.substr(pos));
      
      return (it != fieldsByName.end());
    }      
  }
}

bool MessageFieldCollection::isEmpty() const {
  return fieldsInOrder.empty();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageFieldCollection::appendField(const MessageField& field) {
  fieldsInOrder.push_back(MessageFieldPtr(new MessageField(field)));
  
  std::map<std::string, MessageFieldPtr>::const_iterator it =
    fieldsByName.find(field.getName());
  if (it == fieldsByName.end())
    fieldsByName.insert(std::make_pair(field.getName(),
      fieldsInOrder.back()));
}

void MessageFieldCollection::merge(const MessageFieldCollection& collection) {
  fieldsInOrder.insert(fieldsInOrder.end(), collection.fieldsInOrder.begin(),
    collection.fieldsInOrder.end());
  fieldsByName.insert(collection.fieldsByName.begin(),
    collection.fieldsByName.end());
}

void MessageFieldCollection::clear() {
  fieldsInOrder.clear();
  fieldsByName.clear();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

MessageField& MessageFieldCollection::operator[](size_t index) {
  return getField(index);
}

const MessageField& MessageFieldCollection::operator[](size_t index) const {
  return getField(index);
}

MessageField& MessageFieldCollection::operator[](const std::string& name) {
  return getField(name);
}

const MessageField& MessageFieldCollection::operator[](const std::string& name)
    const {
  return getField(name);
}

MessageFieldCollection& MessageFieldCollection::operator+=(const MessageField&
    field) {
  appendField(field);
  return *this;
}

}
