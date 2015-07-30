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

/** \file MessageDefinition.h
  * \brief Header file providing the MessageDefinition class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_DEFINITION_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_DEFINITION_H

#include <map>
#include <iostream>

#include <boost/enable_shared_from_this.hpp>

#include <ros/ros.h>

#include <variant_topic_tools/Forwards.h>
#include <variant_topic_tools/MessageField.h>
#include <variant_topic_tools/MessageFieldCollection.h>
#include <variant_topic_tools/MessageType.h>

namespace variant_topic_tools {
  /** \brief Message definition
    */
  class MessageDefinition :
    public MessageFieldCollection,
    public boost::enable_shared_from_this<MessageDefinition>{
  public:
    /** \brief Default constructor
      */ 
    MessageDefinition(const MessageType& messageType = MessageType());
    
    /** \brief Copy constructor
      */ 
    MessageDefinition(const MessageDefinition& src);
    
    /** \brief Destructor
      */ 
    ~MessageDefinition();
    
    /** \brief Access the message type represented by this message definition
      */ 
    void setMessageType(const MessageType& type);
    MessageType& getMessageType();
    const MessageType& getMessageType() const;
    
    /** \brief True, if this message definition is valid
      */ 
    bool isValid() const;
    
    /** \brief Load the definition of the message type corresponding to the
      *   specified data type
      */
    void load(const std::string& messageDataType);
    
    /** \brief Clear the message definition
      */
    virtual void clear();
    
    /** \brief Write the message definition to a stream
      */
    void write(std::ostream& stream, const std::string& indent =
      std::string()) const;
    
  protected:
    MessageType messageType;
    
    /** \brief Parse the message definition
      */
    void parse();
    
    /** \brief Recursively fill the fields of the message definition
      */
    void fillFields(const std::map<std::string, MessageFieldCollectionPtr>&
      fields, MessageField& currentField);    
  };
  
  /** \brief Operator for writing the message definition to a stream
    */
  std::ostream& operator<<(std::ostream& stream, const MessageDefinition&
    messageDefinition);
};

#endif
