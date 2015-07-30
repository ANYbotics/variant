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

/** \file MessageField.h
  * \brief Header file providing the MessageField class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_FIELD_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_FIELD_H

#include <ros/ros.h>

#include <variant_topic_tools/MessageFieldCollection.h>
#include <variant_topic_tools/MessageType.h>

namespace variant_topic_tools {
  /** \brief Variant message field
    */
  class MessageField :
    public MessageFieldCollection {
  public:
    /** \brief Default constructor
      */ 
    MessageField(const std::string& name = std::string(), const
      MessageType& type = MessageType(), const std::string&
      value = std::string());
    
    /** \brief Copy constructor
      */ 
    MessageField(const MessageField& src);
    
    /** \brief Destructor
      */ 
    ~MessageField();
    
    /** \brief Access the name of the message field
      */
    void setName(const std::string& name);
    const std::string& getName() const;
    
    /** \brief Access the type of the message field
      */
    void setType(const MessageType& type);
    const MessageType& getType() const;
    
    /** \brief Access the value of the message field
      */
    void setValue(const std::string& value);
    const std::string& getValue() const;
    
    /** \brief True, if this message field is constant
      */ 
    bool isConstant() const;
    
    /** \brief True, if this message field is valid
      */ 
    bool isValid() const;
    
    /** \brief Clear the message field
      */
    virtual void clear();
    
    /** \brief Write the message field to a stream
      */
    void write(std::ostream& stream, const std::string& indent =
      std::string()) const;
    
  protected:
    std::string name;
    MessageType type;
    
    std::string value;
  };
  
  /** \brief Operator for writing the message field to a stream
    */
  std::ostream& operator<<(std::ostream& stream, const MessageField&
    messageField);
};

#endif
