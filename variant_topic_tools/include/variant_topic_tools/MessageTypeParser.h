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

/** \file MessageTypeParser.h
  * \brief Header file providing the MessageTypeParser class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_TYPE_PARSER_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_TYPE_PARSER_H

#include <boost/regex.hpp>

#include <ros/ros.h>

namespace variant_topic_tools {
  /** \brief Message definition parser
    */
  class MessageTypeParser {
  public:
    /** \brief Match any message type expression
      */
    static bool matchType(const std::string& expression, std::string&
      package, std::string& type);
    
  private:
    /** \brief Regular expression for matching a package
      */
    static const boost::regex packageExpression;
    
    /** \brief Regular expression for matching a message type
      */
    static const boost::regex typeExpression;
    
    /** \brief Regular expression for matching a message type without
      *   package prefix
      */
    static const boost::regex plainTypeExpression;
    
    /** \brief Regular expression for matching a message type with
      *   package prefix
      */
    static const boost::regex packageTypeExpression;
    
    /** \brief Default constructor
      */ 
    MessageTypeParser();
    
    /** \brief Destructor
      */ 
    ~MessageTypeParser();    
  };  
};

#endif
