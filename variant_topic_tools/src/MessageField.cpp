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

#include "variant_topic_tools/MessageField.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageField::MessageField(const std::string& name, const MessageType& type,
    const std::string& value) :
  name(name),
  type(type),
  value(value) {
}

MessageField::MessageField(const MessageField& src) :
  MessageFieldCollection(src),
  type(src.type),
  name(src.name),
  value(src.value) {
}

MessageField::~MessageField() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageField::setName(const std::string& name) {
  this->name = name;
}

const std::string& MessageField::getName() const {
  return name;
}

void MessageField::setType(const MessageType& type) {
  this->type = type;
}

const MessageType& MessageField::getType() const {
  return type;
}

void MessageField::setValue(const std::string& value) {
  this->value = value;
}

const std::string& MessageField::getValue() const {
  return value;
}

bool MessageField::isConstant() const {
  return !value.empty();
}

bool MessageField::isValid() const {
  return type.isValid();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageField::clear() {
  MessageFieldCollection::clear();
  
  name.clear();
  type = MessageType();
}

void MessageField::write(std::ostream& stream, const std::string& indent)
    const {
  stream << indent << type.getDataType() << " " << name;
  if (!value.empty())
    stream << " = " << value;
  
  for (size_t i = 0; i < fieldsInOrder.size(); ++i) {
    stream << "\n";
    fieldsInOrder[i]->write(stream, indent+"  ");
  }
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

std::ostream& operator<<(std::ostream& stream, const MessageField&
    messageField) {
  messageField.write(stream);
  return stream;
}

}
