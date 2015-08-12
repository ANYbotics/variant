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

#include <variant_topic_tools/Exceptions.h>
#include <variant_topic_tools/MessageField.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
MessageFieldCollection<T>::MessageFieldCollection() {
}

template <typename T>
MessageFieldCollection<T>::MessageFieldCollection(const
    MessageFieldCollection<T>& src) :
  fieldsInOrder(src.fieldsInOrder),
  fieldsByName(src.fieldsByName) {
}

template <typename T>
MessageFieldCollection<T>::~MessageFieldCollection() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
size_t MessageFieldCollection<T>::getNumFields() const {
  return this->fieldsInOrder.size();
}

template <typename T>
MessageField<T>& MessageFieldCollection<T>::getField(size_t index) {
  if (index < this->fieldsInOrder.size())
    return *(this->fieldsInOrder[index]);
  else
    throw NoSuchMessageFieldException(index);
}

template <typename T>
const MessageField<T>& MessageFieldCollection<T>::getField(size_t index)
    const {
  if (index < this->fieldsInOrder.size())
    return *(this->fieldsInOrder[index]);
  else
    throw NoSuchMessageFieldException(index);
}

template <typename T>
MessageField<T>& MessageFieldCollection<T>::getField(const std::string&
    name) {
  return this->getField(name, 0);
}

template <typename T>
const MessageField<T>& MessageFieldCollection<T>::getField(const std::string&
    name) const {
  return this->getField(name, 0);
}

template <typename T>
MessageField<T>& MessageFieldCollection<T>::getField(const std::string& name,
    size_t pos) const {
  pos = name.find_first_not_of('/', pos);
  
  if (pos != std::string::npos) {
    size_t i = name.find_first_of('/', pos);
    
    if (i != std::string::npos) {
      typename boost::unordered_map<std::string, MessageFieldPtr>::
        const_iterator it = this->fieldsByName.find(name.substr(pos, i-pos));
        
      if (it != this->fieldsByName.end())
        return it->second->getField(name, i+1);
    }
    else {
      typename boost::unordered_map<std::string, MessageFieldPtr>::
        const_iterator it = this->fieldsByName.find(name.substr(pos));
      
      if (it != this->fieldsByName.end())
        return *(it->second);
    }      
  }

  throw NoSuchMessageFieldException(name);
}

template <typename T>
bool MessageFieldCollection<T>::hasField(const std::string& name) const {
  return this->hasField(name, 0);
}

template <typename T>
bool MessageFieldCollection<T>::hasField(const std::string& name, size_t pos)
    const {
  pos = name.find_first_not_of('/', pos);
  
  if (pos != std::string::npos) {
    size_t i = name.find_first_of('/', pos);
    
    if (i != std::string::npos) {
      typename boost::unordered_map<std::string, MessageFieldPtr>::
        const_iterator it = this->fieldsByName.find(name.substr(
        pos, i-pos));
        
      return ((it != this->fieldsByName.end()) &&
        it->second->hasField(name, i+1));
    }
    else {
      typename boost::unordered_map<std::string, MessageFieldPtr>::
        const_iterator it = this->fieldsByName.find(name.substr(pos));
      
      return (it != this->fieldsByName.end());
    }      
  }
}

template <typename T>
bool MessageFieldCollection<T>::isEmpty() const {
  return this->fieldsInOrder.empty();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T>
void MessageFieldCollection<T>::appendField(const MessageField<T>& field) {
  BOOST_ASSERT(field.getName().find('/') == std::string::npos);
  
  typename boost::unordered_map<std::string, MessageFieldPtr>::const_iterator
    it = this->fieldsByName.find(field.getName());
    
  if (it == this->fieldsByName.end()) {
    this->fieldsInOrder.push_back(MessageFieldPtr(new MessageField<T>(field)));
    this->fieldsByName.insert(std::make_pair(field.getName(),
      this->fieldsInOrder.back()));
  }
  else
    throw AmbiguousMessageFieldNameException(field.getName());
}

template <typename T>
void MessageFieldCollection<T>::appendField(const std::string& name,
    const T& value) {
  this->appendField(MessageField<T>(name, value));
}

template <typename T>
void MessageFieldCollection<T>::merge(const MessageFieldCollection<T>&
    collection) {
  this->fieldsInOrder.insert(this->fieldsInOrder.end(),
    collection.fieldsInOrder.begin(), collection.fieldsInOrder.end());
  this->fieldsByName.insert(collection.fieldsByName.begin(),
    collection.fieldsByName.end());
}

template <typename T>
void MessageFieldCollection<T>::clear() {
  this->fieldsInOrder.clear();
  this->fieldsByName.clear();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

template <typename T>
MessageField<T>& MessageFieldCollection<T>::operator[](size_t index) {
  return this->getField(index);
}

template <typename T>
const MessageField<T>& MessageFieldCollection<T>::operator[](size_t index)
    const {
  return this->getField(index);
}

template <typename T>
MessageField<T>& MessageFieldCollection<T>::operator[](const std::string&
    name) {
  return this->getField(name);
}

template <typename T>
const MessageField<T>& MessageFieldCollection<T>::operator[](const
    std::string& name) const {
  return this->getField(name);
}

template <typename T>
MessageFieldCollection<T>& MessageFieldCollection<T>::operator+=(const
    MessageField<T>& field) {
  this->appendField(field);
  return *this;
}

}
