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

#include <list>
#include <set>

#include "variant_topic_tools/DataTypeRegistry.h"
#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/MessageDefinition.h"
#include "variant_topic_tools/MessageDefinitionParser.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageDefinition::MessageDefinition() {
}

MessageDefinition::MessageDefinition(const MessageType& messageType) {
  setMessageType(messageType);
}

MessageDefinition::MessageDefinition(const MessageDataType& messageDataType) :
  messageDataType(messageDataType) {
}

MessageDefinition::MessageDefinition(const MessageDefinition& src) :
  MessageFieldCollection(src),
  messageDataType(src.messageDataType) {
}

MessageDefinition::~MessageDefinition() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageDefinition::setMessageType(const MessageType& messageType) {
  clear();
  
  if (messageType.isValid()) {
    DataType dataType(messageType.getDataType());
    
    if (!dataType.isValid()) {
      DataTypeRegistry registry;
      std::vector<MessageType> messageTypes;

      MessageDefinitionParser::parse(messageType.getDataType(),
        messageType.getDefinition(), messageTypes);
      
      for (size_t i = messageTypes.size(); i > 0; --i) {
        if (!registry.getDataType(messageTypes[i-1].getDataType()).isValid())
          registry.addMessageDataType(messageTypes[i-1].getDataType(),
            messageTypes[i-1].getDefinition());
      }
    }

    messageDataType = MessageDataType(messageType.getDataType());
    fill(messageDataType, *this);
  }
}

void MessageDefinition::setMessageDataType(const MessageDataType&
    messageDataType) {
  setMessageType(messageDataType);
}

const MessageDataType& MessageDefinition::getMessageDataType() const {
  return messageDataType;
}

bool MessageDefinition::isValid() const {
  return messageDataType.isValid();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageDefinition::load(const std::string& messageDataType) {
  clear();
  
  MessageType messageType;
  messageType.load(messageDataType);
  
  setMessageType(messageType);
}

void MessageDefinition::clear() {
  MessageFieldCollection::clear();
  messageDataType = MessageDataType();
}

void MessageDefinition::fill(const MessageDataType& currentDataType,
    MessageFieldCollection<DataType>& currentCollection) {
  for (size_t i = 0; i < currentDataType.getNumMembers(); ++i) {
    currentCollection.appendField(currentDataType[i].getName(), 
      currentDataType[i].getType());
    
    if (currentDataType[i].getType().isMessage())
      fill(currentDataType[i].getType(), currentCollection[i]);
  }
}

void MessageDefinition::write(std::ostream& stream) const {
  stream << messageDataType.getDefinition();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

std::ostream& operator<<(std::ostream& stream, const MessageDefinition&
    messageDefinition) {
  messageDefinition.write(stream);
  return stream;
}

}
