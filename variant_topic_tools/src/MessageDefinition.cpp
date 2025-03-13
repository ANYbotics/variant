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

#include <boost/unordered_map.hpp>

#include "variant_topic_tools/DataTypeRegistry.h"
#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/MessageDataType.h"
#include "variant_topic_tools/MessageDefinition.h"
#include "variant_topic_tools/MessageDefinitionParser.h"
#include "variant_topic_tools/MessageTypeParser.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageDefinition::MessageDefinition() = default;

MessageDefinition::MessageDefinition(const MessageType& messageType) {
  setMessageType(messageType);
}

MessageDefinition::MessageDefinition(const MessageDataType& messageDataType) : messageDataType(messageDataType) {}

MessageDefinition::MessageDefinition(const MessageDefinition& src) : MessageFieldCollection(src), messageDataType(src.messageDataType) {}

MessageDefinition::~MessageDefinition() = default;

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
      boost::unordered_map<std::string, size_t> definedTypes;
      boost::unordered_map<size_t, std::set<size_t> > requiredTypes;

      MessageDefinitionParser::parse(messageType.getDataType(), messageType.getDefinition(), messageTypes);

      for (size_t i = 0; i < messageTypes.size(); ++i) {
        definedTypes[messageTypes[i].getDataType()] = i;
      }

      for (size_t i = 0; i < messageTypes.size(); ++i) {
        std::string package;
        std::string plainType;

        if (!MessageTypeParser::matchType(messageTypes[i].getDataType(), package, plainType)) {
          throw InvalidMessageTypeException(messageTypes[i].getDataType());
        }

        std::istringstream stream(messageTypes[i].getDefinition());
        std::string line;

        while (std::getline(stream, line)) {
          std::string memberName;
          std::string memberType;
          size_t memberSize = 0;

          if (MessageDefinitionParser::matchArray(line, memberName, memberType, memberSize) ||
              MessageDefinitionParser::match(line, memberName, memberType)) {
            std::string memberPackage;
            std::string plainMemberType;

            if (!MessageTypeParser::matchType(memberType, memberPackage, plainMemberType)) {
              throw InvalidMessageTypeException(memberType);
            }

            if (memberPackage.empty()) {
              if (plainMemberType == "Header") {
                memberPackage = "std_msgs";
              } else {
                memberPackage = package;
              }

              memberType = memberPackage + "/" + plainMemberType;
            }

            boost::unordered_map<std::string, size_t>::iterator it = definedTypes.find(memberType);

            if (it != definedTypes.end()) {
              requiredTypes[i].insert(it->second);
            }
          }
        }
      }

      std::list<size_t> typesToBeDefined;
      typesToBeDefined.push_back(definedTypes[messageType.getDataType()]);

      while (!typesToBeDefined.empty()) {
        size_t i = typesToBeDefined.back();
        const std::set<size_t>& currentRequiredTypes = requiredTypes[i];
        bool allRequiredTypesDefined = true;

        for (unsigned long currentRequiredType : currentRequiredTypes) {
          if (!registry.getDataType(messageTypes[currentRequiredType].getDataType()).isValid()) {
            typesToBeDefined.push_back(currentRequiredType);
            allRequiredTypesDefined = false;

            break;
          }
        }

        if (allRequiredTypesDefined) {
          registry.addMessageDataType(messageTypes[i].getDataType(), messageTypes[i].getDefinition());
          typesToBeDefined.pop_back();
        }
      }
    }

    messageDataType = MessageDataType(messageType.getDataType());
    fill(messageDataType, *this);
  }
}

void MessageDefinition::setMessageDataType(const MessageDataType& messageDataType) {
  setMessageType(messageDataType);
}

MessageDataType MessageDefinition::getMessageDataType() const {
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

void MessageDefinition::fill(const MessageDataType& currentDataType, MessageFieldCollection<DataType>& currentCollection) {
  for (size_t i = 0; i < currentDataType.getNumMembers(); ++i) {
    currentCollection.appendField(currentDataType[i].getName(), currentDataType[i].getType());

    if (currentDataType[i].getType().isMessage()) {
      fill(currentDataType[i].getType(), currentCollection[i]);
    }
  }
}

void MessageDefinition::write(std::ostream& stream) const {
  stream << MessageDataType(messageDataType).getDefinition();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

std::ostream& operator<<(std::ostream& stream, const MessageDefinition& messageDefinition) {
  messageDefinition.write(stream);
  return stream;
}

}  // namespace variant_topic_tools
