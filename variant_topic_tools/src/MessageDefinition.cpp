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

MessageDefinition::MessageDefinition(const MessageDataType& messageDataType) {
  setMessageDataType(messageDataType);
}

MessageDefinition::MessageDefinition(const MessageDefinition& src) :
  MessageFieldCollection(src),
  messageType(src.messageType),
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
    this->messageType = messageType;
    
    try {
      parse();
    }
    catch (const DefinitionParseException& exception) {
      clear();
      throw exception;
    }
  }
}

const MessageType& MessageDefinition::getMessageType() const {
  return messageType;
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
  
  messageType = MessageType();
}

void MessageDefinition::parse() {
  const std::string& messageDataType = messageType.getDataType();
  const std::string& messageDefinition = messageType.getDefinition();
  std::map<std::string, MessageFieldCollectionPtr> fields;  
  MessageFieldCollectionPtr messageFields(new MessageFieldCollection());  
  MessageFieldCollectionPtr currentFields = messageFields;
  size_t pos = 0;
  
  while (pos != std::string::npos) {
    size_t i = messageDefinition.find_first_of('\n', pos);
    
    size_t lineLength = i;
    if (i != std::string::npos)
      lineLength = i-pos;
    
    std::string line = messageDefinition.substr(pos, lineLength);
    
    if (i != std::string::npos)
      pos = i+1;
    else
      pos = std::string::npos;
    
    const boost::regex emptyExpression("\\h*");
    const boost::regex commentExpression("\\h*#\\h*(.*)");
    const boost::regex separatorExpression("=+");
    const boost::regex variableExpression("\\h*(\\H+)\\h+(\\H+).*");
    const boost::regex constantExpression(
      "\\h*(\\H+)\\h+(\\H+)\\h*=\\h*(\\H+).*");
    const boost::regex dependencyExpression("\\h*MSG:\\h*(\\H+).*");
    boost::smatch match;
    
    if (boost::regex_match(line, match, emptyExpression) ||
        boost::regex_match(line, match, commentExpression) ||
        boost::regex_match(line, match, separatorExpression))
      continue;
    else if (boost::regex_match(line, match, dependencyExpression)) {
      std::string dependencyDataType(match[1].first, match[1].second);
      
      currentFields.reset(new MessageFieldCollection());
      fields.insert(std::make_pair(dependencyDataType, currentFields));
    }
    else if (boost::regex_match(line, match, constantExpression)) {
      std::string constantType(match[1].first, match[1].second);
      std::string constantName(match[2].first, match[2].second);
      std::string constantValue(match[3].first, match[3].second);

      MessageField field(constantName, constantType, constantValue);
      currentFields->appendField(field);
    }
    else if (boost::regex_match(line, match, variableExpression)) {
      std::string variableType(match[1].first, match[1].second);
      std::string variableName(match[2].first, match[2].second);
      
      if (variableType == "Header")
        variableType = "std_msgs/Header";
      
      MessageField field(variableName, variableType);
      currentFields->appendField(field);
    }
    else
      throw DefinitionParseException(messageDataType, line, 
        "Line has unexpected format");    
  }
  
  for (size_t i = 0; i < messageFields->getNumFields(); ++i)
    fillFields(fields, messageFields->getField(i));
  merge(*messageFields);
}

void MessageDefinition::fillFields(const std::map<std::string,
    MessageFieldCollectionPtr>& fields, MessageField& currentField) {
  std::map<std::string, MessageFieldCollectionPtr>::const_iterator it =
    fields.find(currentField.getType().getDataType());
  
  if (it != fields.end()) {
    currentField.merge(*(it->second));
    
    for (size_t i = 0; i < currentField.getNumFields(); ++i)
      fillFields(fields, currentField[i]);
  }
}

void MessageDefinition::write(std::ostream& stream, const std::string& indent)
    const {
  messageType.write(stream, indent);
  stream << "\n";
  stream << indent << "Message definition:";
  
  for (size_t i = 0; i < fieldsInOrder.size(); ++i) {
    stream << "\n";
    fieldsInOrder[i]->write(stream, indent+"  ");
  }
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
