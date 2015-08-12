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

#include <fstream>

#include <ros/package.h>

#include <boost/regex.hpp>

#include "variant_topic_tools/DataTypeRegistry.h"
#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/MessageDefinition.h"

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
    try {
      parse(messageType.getDataType(), messageType.getDefinition());
    }
    catch (const DefinitionParseException& exception) {
      clear();
      throw exception;
    }
    
    messageDataType = MessageDataType(messageType.getDataType());
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
  std::string package, type;
  
  size_t i = messageDataType.find_first_of('/');
  if ((i > 0) && (i != std::string::npos)) {
    package = messageDataType.substr(0, i-1);
    type = messageDataType.substr(i+1);
  }
  else
    type = messageDataType;
  
  if (package.empty()) {
    if (type == "Header")
      package = "std_msgs";
    else
      throw InvalidMessageTypeException(messageDataType);
  }
  
  if (type.empty())
    throw InvalidDataTypeException();
  
  std::string packagePath = ros::package::getPath(package);
  if (packagePath.empty())
    throw PackageNotFoundException(package);
  
  std::string messageFilename(package+"/"+type+".msg");
  std::ifstream messageFile(messageFilename.c_str());

  if (messageFile.is_open()) {
    std::string messageDefinition;
    
    messageFile.seekg(0, std::ios::end);   
    messageDefinition.reserve(messageFile.tellg());
    messageFile.seekg(0, std::ios::beg);
    
    messageDefinition.assign((std::istreambuf_iterator<char>(messageFile)),
      std::istreambuf_iterator<char>());
  }
  else
    throw FileOpenException(messageFilename);
  
  messageFile.close();
}

void MessageDefinition::clear() {
  MessageFieldCollection::clear();
  messageDataType = MessageDataType();
}

void MessageDefinition::parse(const std::string& messageDataType, const
    std::string& messageDefinition) {
  BOOST_ASSERT(!messageDefinition.empty());
  
  std::vector<std::string> messageTypes;
  std::vector<std::string> messageDefinitions;
  
  const boost::regex separatorExpression("=+");
  const boost::regex messageTypeExpression(
    "^\\h*MSG:\\h*([a-zA-Z][a-zA-Z1-9_/]*).*$");
  
  std::istringstream stream(messageDefinition);
  std::string currentMessageType = messageDataType;
  std::string currentMessageDefinition;
  std::string line;  
  
  while (std::getline(stream, line)) {
    boost::smatch match;
    
    if (boost::regex_match(line, match, messageTypeExpression)) {
      if (!currentMessageDefinition.empty()) {
        messageTypes.push_back(currentMessageType);
        messageDefinitions.push_back(currentMessageDefinition);
      }
      
      currentMessageType = std::string(match[1].first, match[1].second);
      currentMessageDefinition.clear();      
    }
    else if (!boost::regex_match(line, match, separatorExpression))
      currentMessageDefinition += line+"\n";
  }

  if (!currentMessageDefinition.empty()) {
    messageTypes.push_back(currentMessageType);
    messageDefinitions.push_back(currentMessageDefinition);
  }
  
  DataTypeRegistry registry;
  
  for (size_t i = messageTypes.size(); i > 0; --i) {
    if (!registry.getDataType(messageTypes[i-1]).isValid())
      registry.addMessageDataType(messageTypes[i-1],
        messageDefinitions[i-1]);
  }
}

void MessageDefinition::fill(const DataType& currentDataType,
    MessageField<DataType>& currentField) {
}

// void MessageDefinition::fill(const std::map<std::string,
//     MessageFieldCollectionPtr>& fields, MessageField& currentField) {
//   std::map<std::string, MessageFieldCollectionPtr>::const_iterator it =
//     fields.find(currentField.getType().getDataType());
//   
//   if (it != fields.end()) {
//     currentField.merge(*(it->second));
//     
//     for (size_t i = 0; i < currentField.getNumFields(); ++i)
//       fill(fields, currentField[i]);
//   }
// }

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
