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

#include "variant_topic_tools/MessageTypeParser.h"

namespace variant_topic_tools {
  
/*****************************************************************************/
/* Static initializers                                                       */
/*****************************************************************************/

const boost::regex MessageTypeParser::packageExpression =
  boost::regex("[a-zA-Z][a-zA-Z1-9_]*");

const boost::regex MessageTypeParser::typeExpression =
  boost::regex("[a-zA-Z][a-zA-Z1-9_]*");
  
const boost::regex MessageTypeParser::plainTypeExpression =
  boost::regex("("+typeExpression.str()+")");
  
const boost::regex MessageTypeParser::packageTypeExpression =
  boost::regex("("+packageExpression.str()+")/("+typeExpression.str()+")");
  
/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageTypeParser::MessageTypeParser() {
}

MessageTypeParser::~MessageTypeParser() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool MessageTypeParser::matchType(const std::string& expression, std::string&
    package, std::string& type) {
  boost::smatch match;
  
  if (boost::regex_match(expression, match, packageTypeExpression)) {
    package = std::string(match[1].first, match[1].second);
    type = std::string(match[2].first, match[2].second);
    
    return true;
  }
  else if (boost::regex_match(expression, match, plainTypeExpression)) {
    package = std::string();
    type = std::string(match[1].first, match[1].second);
    
    return true;
  }
  else
    return false;
}

}
