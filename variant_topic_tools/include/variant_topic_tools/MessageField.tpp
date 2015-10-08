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

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
MessageField<T>::MessageField(const std::string& name, const T& value) :
  name(name),
  value(value) {
}

template <typename T>
MessageField<T>::MessageField(const MessageField& src) :
  MessageFieldCollection<T>(src),
  name(src.name),
  value(src.value) {
}

template <typename T>
MessageField<T>::~MessageField() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
void MessageField<T>::setName(const std::string& name) {
  this->name = name;
}

template <typename T>
const std::string& MessageField<T>::getName() const {
  return this->name;
}

template <typename T>
void MessageField<T>::setValue(const T& value) {
  this->value = value;
}

template <typename T>
T& MessageField<T>::getValue() {
  return this->value;
}

template <typename T>
const T& MessageField<T>::getValue() const {
  return this->value;
}

template <typename T>
bool MessageField<T>::isValid() const {
  return MessageField<T>::template isValid<T>(value);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T>
void MessageField<T>::clear() {
  MessageFieldCollection<T>::clear();
  
  this->name.clear();
  this->value = T();
}

template <typename T>
void MessageField<T>::write(std::ostream& stream, const std::string& indent)
    const {
  stream << indent << name << ": ";

  std::stringstream valueStream;
  valueStream << value;
  std::string valueLine;
  size_t numLines = 0;
  
  while (std::getline(valueStream, valueLine)) {
    if (!valueStream.eof() || numLines)
      stream << "\n" << indent << "  ";
    
    stream << valueLine;
    ++numLines;
  }

  if (!this->fieldsInOrder.empty()) {
    stream << "\n";
    MessageFieldCollection<T>::write(stream, indent+"  ");
  }
}

template <typename T>
template <typename U> bool MessageField<T>::isValid(const U& value, typename
    boost::enable_if_c<MessageFieldTypeTraits::HasIsValid<U>::value>::type*) {
  return value.isValid();
}

template <typename T>
template <typename U> bool MessageField<T>::isValid(const U& value, typename
    boost::disable_if_c<MessageFieldTypeTraits::HasIsValid<U>::value>::type*) {
  return true;
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

template <typename T>
bool MessageField<T>::operator==(const MessageField<T>& field) const {
  return (this->name == field.name) && (this->value == field.value) &&
    MessageFieldCollection<T>::operator==(field);
}

template <typename T>
bool MessageField<T>::operator!=(const MessageField<T>& field) const {
  return (this->name != field.name) || (this->value != field.value) ||
    MessageFieldCollection<T>::operator!=(field);
}

template <typename T> std::ostream& operator<<(std::ostream& stream, const
    MessageField<T>& messageField) {
  messageField.write(stream);
  return stream;
}

}
