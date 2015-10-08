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

/** \file MessageMemberPointer.h
  * \brief Header file providing the MessageMemberPointer class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_MEMBER_POINTER_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_MEMBER_POINTER_H

#include <variant_topic_tools/Pointer.h>

namespace variant_topic_tools {
  /** \brief Shared pointer
    */
  template <class M, typename T> class MessageMemberPointer :
    public Pointer<T> {
  public:
    /** \brief Definition of the message pointer type
      */
    typedef Pointer<M> MessagePtr;
      
    /** \brief Default constructor
      */ 
    MessageMemberPointer(M* message = 0, size_t offset = 0);
    
    /** \brief Constructor (overloaded version taking a message pointer
      *   and an offset)
      */ 
    MessageMemberPointer(const MessagePtr& message, size_t offset);
    
    /** \brief Copy constructor
      */ 
    MessageMemberPointer(const MessageMemberPointer<M, T>& src);
    
    /** \brief Copy constructor (overloaded version taking a pointer)
      */ 
    MessageMemberPointer(const Pointer<T>& src);
    
    /** \brief Destructor
      */ 
    ~MessageMemberPointer();
    
    /** \brief Set the message
      */
    void setMessage(const MessagePtr& message);
    
    /** \brief Retrieve the message
      */
    const MessagePtr& getMessage() const;
    
    /** \brief Set the message member offset
      */
    void setOffset(size_t offset);
    
    /** \brief Retrieve the message member offset
      */
    size_t getOffset() const;

  protected:
    /** \brief Message member pointer implementation
      */
    class Impl :
      public Pointer<T>::Impl {
    public:
      /** \brief Constructor
        */
      Impl(const MessagePtr& message = MessagePtr(), size_t
        offset = 0);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the stored pointer (implementation)
        */
      T* get() const;
      
      /** \brief The shared message
        */ 
      MessagePtr message;
      
      /** \brief The member offset
        */ 
      size_t offset;
    };
  };
};

#include <variant_topic_tools/MessageMemberPointer.tpp>

#endif
