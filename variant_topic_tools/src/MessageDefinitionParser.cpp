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

#include "variant_topic_tools/MessageDefinitionParser.h"

namespace variant_topic_tools {
  
/*****************************************************************************/
/* Static initializers                                                       */
/*****************************************************************************/

const boost::regex MessageDefinitionParser::commentExpression =
  boost::regex("#.*$");
  
const boost::regex MessageDefinitionParser::separatorExpression =
  boost::regex("^==+$");

const boost::regex MessageDefinitionParser::messageTypeExpression =
  boost::regex("^\\h*MSG:\\h*([a-zA-Z][a-zA-Z1-9_/]*).*$");

const boost::regex MessageDefinitionParser::memberNameExpression =
  boost::regex("[a-zA-Z][a-zA-Z1-9_]*");

const boost::regex MessageDefinitionParser::memberTypeExpression =
  boost::regex("[a-zA-Z][a-zA-Z1-9_/]*");

const boost::regex MessageDefinitionParser::memberArrayTypeExpression =
  boost::regex("("+memberTypeExpression.str()+")\\[([0-9]*)\\]");

const boost::regex MessageDefinitionParser::memberValueExpression =
  boost::regex("[^\\h]+");

const boost::regex MessageDefinitionParser::memberExpression =
  boost::regex("^\\h*("+memberTypeExpression.str()+")(\\[[0-9]*\\])?\\h+("+
  memberNameExpression.str()+").*$");

const boost::regex MessageDefinitionParser::constantMemberExpression =
  boost::regex("^\\h*("+memberTypeExpression.str()+")\\h+("+
    memberNameExpression.str()+")\\h*=\\h*("+memberValueExpression.str()+
    ")\\h*("+commentExpression.str()+")?$");

const boost::regex MessageDefinitionParser::constantStringMemberExpression =
  boost::regex("^\\h*(string)\\h+("+memberNameExpression.str()+
  ")\\h*=\\h*(.*?)\\h*$");

const boost::regex MessageDefinitionParser::variableMemberExpression =
  boost::regex("^\\h*("+memberTypeExpression.str()+")(\\[[0-9]*\\])?\\h+("+
    memberNameExpression.str()+")\\h*("+commentExpression.str()+")?$");

const boost::regex MessageDefinitionParser::arrayMemberExpression =
  boost::regex("^\\h*"+memberArrayTypeExpression.str()+"\\h+("+
    memberNameExpression.str()+")\\h*("+commentExpression.str()+")?$");

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageDefinitionParser::MessageDefinitionParser() {
}

MessageDefinitionParser::~MessageDefinitionParser() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

size_t MessageDefinitionParser::parse(const std::string& messageDataType, const
    std::string& messageDefinition, std::vector<MessageType>& messageTypes) {
  messageTypes.clear();  
  
  std::istringstream stream(messageDefinition);
  std::string currentMessageType = messageDataType;
  std::string currentMessageDefinition;
  std::string line;  
  
  while (std::getline(stream, line)) {
    boost::smatch match;
    
    if (boost::regex_match(line, match, messageTypeExpression)) {
      if (!currentMessageDefinition.empty())
        messageTypes.push_back(MessageType(currentMessageType, "*", 
          currentMessageDefinition));
      
      currentMessageType = std::string(match[1].first, match[1].second);
      currentMessageDefinition.clear();      
    }
    else if (!boost::regex_match(line, match, separatorExpression)) {
      if (!currentMessageDefinition.empty())
        currentMessageDefinition += "\n";
      currentMessageDefinition += line;
    }
  }

  if (!currentMessageDefinition.empty())
    messageTypes.push_back(MessageType(currentMessageType, "*",
      currentMessageDefinition));
    
  return messageTypes.size();
}

bool MessageDefinitionParser::matchType(const std::string& expression) {
  boost::smatch match;
  return boost::regex_match(expression, match, memberTypeExpression);
}

bool MessageDefinitionParser::matchArrayType(const std::string& expression,
    std::string& elementType, size_t& size) {
  boost::smatch match;
  
  if (boost::regex_match(expression, match, memberArrayTypeExpression)) {
    elementType = std::string(match[1].first, match[1].second);
    
    if (match[2].first != match[2].second)
      size = boost::lexical_cast<size_t>(
        std::string(match[2].first, match[2].second));
    else
      size = 0;

    return true;
  }
  else
    return false;
}

bool MessageDefinitionParser::match(const std::string& expression, std::string&
    name, std::string& type) {
  boost::smatch match;
  
  if (boost::regex_match(expression, match, memberExpression)) {
    name = std::string(match[3].first, match[3].second);
    type = std::string(match[1].first, match[1].second)+
      std::string(match[2].first, match[2].second);
    
    return true;
  }
  else
    return false;
}

bool MessageDefinitionParser::matchConstant(const std::string& expression,
    std::string& name, std::string& type, std::string& value) {
  boost::smatch match;
  
  if (boost::regex_match(expression, match, constantStringMemberExpression) ||
      (boost::regex_match(expression, match, constantMemberExpression))) {
    name = std::string(match[2].first, match[2].second);
    type = std::string(match[1].first, match[1].second);
    value = std::string(match[3].first, match[3].second);
    
    return true;
  }
  else
    return false;
}

bool MessageDefinitionParser::matchVariable(const std::string& expression,
    std::string& name, std::string& type) {
  boost::smatch match;
  
  if (boost::regex_match(expression, match, variableMemberExpression)) {
    name = std::string(match[3].first, match[3].second);
    type = std::string(match[1].first, match[1].second)+
      std::string(match[2].first, match[2].second);
    
    return true;
  }
  else
    return false;
}

bool MessageDefinitionParser::matchArray(const std::string& expression,
    std::string& name, std::string& elementType, size_t& size) {
  boost::smatch match;
  
  if (boost::regex_match(expression, match, arrayMemberExpression)) {
    name = std::string(match[3].first, match[3].second);
    elementType = std::string(match[1].first, match[1].second);
    
    if (match[2].first != match[2].second)
      size = boost::lexical_cast<size_t>(
        std::string(match[2].first, match[2].second));
    else
      size = 0;
    
    return true;
  }
  else
    return false;
}

bool MessageDefinitionParser::matchSeparator(const std::string& expression) {
  boost::smatch match;
  
  return boost::regex_match(expression, match, separatorExpression);
}

}
