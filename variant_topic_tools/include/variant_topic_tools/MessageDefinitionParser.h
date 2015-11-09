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

/** \file MessageDefinitionParser.h
  * \brief Header file providing the MessageDefinitionParser class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_DEFINITION_PARSER_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_DEFINITION_PARSER_H

#include <vector>

#include <boost/regex.hpp>

#include <ros/ros.h>

#include <variant_topic_tools/MessageType.h>

namespace variant_topic_tools {
  /** \brief Message definition parser
    */
  class MessageDefinitionParser {
  public:
    /** \brief Parse a message definition and generate the message types
      */
    static size_t parse(const std::string& messageDataType, const std::string&
      messageDefinition, std::vector<MessageType>& messageTypes);
        
    /** \brief Match any message type expression
      */
    static bool matchType(const std::string& expression);
    
    /** \brief Match an array type expression
      */
    static bool matchArrayType(const std::string& expression, std::string&
      memberType, size_t& size);
    
    /** \brief Match any message member expression
      */
    static bool match(const std::string& expression, std::string& name,
      std::string& type);
    
    /** \brief Match a constant message member expression
      */
    static bool matchConstant(const std::string& expression, std::string&
      name, std::string& type, std::string& value);
    
    /** \brief Match a variable message member expression
      */
    static bool matchVariable(const std::string& expression, std::string&
      name, std::string& type);
    
    /** \brief Match an array message member expression
      */
    static bool matchArray(const std::string& expression, std::string& name,
      std::string& memberType, size_t& size);
  
    /** \brief Match a separator expression
      */
    static bool matchSeparator(const std::string& expression);
    
  private:
    /** \brief Regular expression for matching a comment
      */
    static const boost::regex commentExpression;
    
    /** \brief Regular expression for matching a message type separator
      */
    static const boost::regex separatorExpression;
    
    /** \brief Regular expression for matching a message type definition
      */
    static const boost::regex messageTypeExpression;
    
    /** \brief Regular expression for matching a message member name
      */
    static const boost::regex memberNameExpression;
    
    /** \brief Regular expression for matching any message member type
      */
    static const boost::regex memberTypeExpression;
    
    /** \brief Regular expression for matching an array message member type
      */
    static const boost::regex memberArrayTypeExpression;
    
    /** \brief Regular expression for matching a message member value
      */
    static const boost::regex memberValueExpression;
    
    /** \brief Regular expression for matching any message member
      */
    static const boost::regex memberExpression;
    
    /** \brief Regular expression for matching a constant message member
      */
    static const boost::regex constantMemberExpression;
    
    /** \brief Regular expression for matching a constant string message
      *   member
      */
    static const boost::regex constantStringMemberExpression;
    
    /** \brief Regular expression for matching a variable message member
      */
    static const boost::regex variableMemberExpression;
    
    /** \brief Regular expression for matching an array message member
      */
    static const boost::regex arrayMemberExpression;
    
    /** \brief Default constructor
      */ 
    MessageDefinitionParser();
    
    /** \brief Destructor
      */ 
    ~MessageDefinitionParser();    
  };  
};

#endif
