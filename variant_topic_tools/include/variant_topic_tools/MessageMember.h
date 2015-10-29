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

/** \file MessageMember.h
  * \brief Header file providing the MessageMember class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_MEMBER_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_MEMBER_H

#include <variant_topic_tools/Forwards.h>

namespace variant_topic_tools {
  /** \brief Message member
    */
  class MessageMember {
  friend class MessageDataType;
  public:
    /** \brief Default constructor
      */ 
    MessageMember();
    
    /** \brief Copy constructor
      */ 
    MessageMember(const MessageMember& src);
    
    /** \brief Destructor
      */ 
    ~MessageMember();
  
    /** \brief Retrieve the name of this message member
      */
    const std::string& getName() const;
    
    /** \brief Retrieve the type of this message member
      */
    const DataType& getType() const;
    
    /** \brief True, if this message member is a variable member
      */
    bool isVariable() const;
    
    /** \brief True, if this message member is a constant member
      */
    bool isConstant() const;
    
    /** \brief True, if this message member is valid
      */ 
    bool isValid() const;
    
    /** \brief Write the message member to a stream
      */
    void write(std::ostream& stream) const;
    
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl) ? (void*)1 : (void*)0;
    };
    
  protected:
    /** \brief Message member implementation
      */
    class Impl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the type of this message member (abstract
        *   declaration)
        */
      virtual const DataType& getType() const = 0;
      
      /** \brief Write the message member to a stream (abstract declaration)
        */
      virtual void write(std::ostream& stream) const = 0;
    
      /** \brief The name of this message member
        */
      std::string name;
    };
    
    /** \brief Declaration of the message member implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the message member implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The message member's implementation
      */
    ImplPtr impl;
  };
    
  /** \brief Operator for writing the message member to a stream
    */
  std::ostream& operator<<(std::ostream& stream, const MessageMember&
    messageMember);
};

#endif
