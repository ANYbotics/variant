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

#include "variant_topic_tools/MessageType.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageType::MessageType(const std::string& dataType, const std::string&
    md5Sum, const std::string& definition) :
  dataType(dataType),
  md5Sum(md5Sum),
  definition(definition) {
}

MessageType::MessageType(const MessageDataType& dataType) :
  dataType(dataType.getIdentifier()),
  md5Sum(dataType.getMD5Sum()),
  definition(dataType.getDefinition()) {
}

MessageType::MessageType(const MessageType& src) :
  dataType(src.dataType),
  md5Sum(src.md5Sum),
  definition(src.definition) {
}

MessageType::~MessageType() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageType::setDataType(const std::string& dataType) {
  this->dataType = dataType;
}

const std::string& MessageType::getDataType() const {
  return dataType;
}

void MessageType::setMD5Sum(const std::string& md5Sum) {
  this->md5Sum = md5Sum;
}

const std::string& MessageType::getMD5Sum() const {
  return md5Sum;
}

void MessageType::setDefinition(const std::string& definition) {
  this->definition = definition;
}

const std::string& MessageType::getDefinition() const {
  return definition;
}

bool MessageType::isValid() const {
  return !md5Sum.empty() && (md5Sum != "*") && !dataType.empty() &&
    !definition.empty();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageType::write(std::ostream& stream, const std::string& indent)
    const {
  stream << indent << "Message data type: " << dataType << "\n";
  stream << indent << "Message MD5 sum: " << md5Sum << "\n";
  stream << indent << "Message definition:\n" << definition;
}
      
/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

bool MessageType::operator==(const MessageType& type) const {
  return (dataType == type.dataType) && (md5Sum == type.md5Sum);
}

bool MessageType::operator!=(const MessageType& type) const {
  return (dataType != type.dataType) || (md5Sum != type.md5Sum);
}

std::ostream& operator<<(std::ostream& stream, const MessageType&
    messageType) {
  messageType.write(stream);
}

}
