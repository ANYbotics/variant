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

/** \file MessageVariable.h
  * \brief Header file providing the MessageVariable class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_VARIABLE_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_VARIABLE_H

#include <variant_topic_tools/DataType.h>
#include <variant_topic_tools/MessageMember.h>

namespace variant_topic_tools {
  /** \brief Variable message member
    */
  class MessageVariable :
    public MessageMember {
  friend class MessageDataType;
  friend class MessageMember;
  public:
    /** \brief Default constructor
      */ 
    MessageVariable();
    
    /** \brief Copy constructor
      */ 
    MessageVariable(const MessageVariable& src);
    
    /** \brief Copy constructor (overloaded version taking a message member)
      */ 
    MessageVariable(const MessageMember& src);
    
    /** \brief Destructor
      */ 
    ~MessageVariable();
    
  protected:
    /** \brief Message variable implementation
      */
    class Impl :
      public MessageMember::Impl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name, const DataType& type);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the type of this message member (implementation)
        */
      const DataType& getType() const;
    
      /** \brief Retrieve the size of this message member (implementation)
        */
      size_t getSize() const;
      
      /** \brief True, if this message member represents a fixed-size
        *   message member, as opposed to a variable-size message member
        *   (implementation)
        */ 
      bool isFixedSize() const;
      
      /** \brief Write the message member to a stream (implementation)
        */
      void write(std::ostream& stream) const;
      
      /** \brief The data type of this message variable
        */
      DataType type;
    };
    
    /** \brief Constructor (overloaded version taking a name and a data type)
      */ 
    MessageVariable(const std::string& name, const DataType& type);
    
    /** \brief Create a message variable
      */ 
    template <typename T> static MessageVariable create(const std::string&
      name);
  };
};

#include <variant_topic_tools/MessageVariable.tpp>

#endif
