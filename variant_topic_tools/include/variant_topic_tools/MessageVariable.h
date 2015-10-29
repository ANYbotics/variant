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
#include <variant_topic_tools/MessageTypeTraits.h>
#include <variant_topic_tools/Pointer.h>

namespace variant_topic_tools {
  /** \brief Variable message member
    */
  class MessageVariable :
    public MessageMember {
  friend class MessageDataType;
  friend class MessageMember;
  friend class MessageVariant;
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
    
      /** \brief Write the message member to a stream (implementation)
        */
      void write(std::ostream& stream) const;
      
      /** \brief The data type of this message variable
        */
      DataType type;
    };
    
    /** \brief Message variable implementation (version templated on the
      *   message type)
      */
    template <typename T> class ImplT :
      public Impl {
    public:
      BOOST_STATIC_ASSERT(type_traits::IsMessage<T>::value);
      
      /** \brief Definition of the value type
        */
      typedef typename type_traits::MessageType<T>::ValueType ValueType;
      
      /** \brief Constructor
        */
      ImplT(const std::string& name, const DataType& type);
      
      /** \brief Destructor
        */
      virtual ~ImplT();
      
      /** \brief Create a variant from this variable message member (abstract
        *   declaration)
        */ 
      virtual Variant createVariant(const Pointer<ValueType>& message)
        const = 0;
    };
    
    /** \brief Message variable implementation (version templated on the
      *   message type and the member type)
      */
    template <typename T, typename M> class ImplM :
      public ImplT<T> {
    public:
      /** \brief Definition of the value type
        */
      typedef typename ImplT<T>::ValueType ValueType;
      
      /** \brief Constructor
        */
      ImplM(const std::string& name, const DataType& type, size_t offset);
      
      /** \brief Destructor
        */
      virtual ~ImplM();
      
      /** \brief Create a variant from this variable message member
        *   (implementation)
        */ 
      Variant createVariant(const Pointer<ValueType>& message) const;
      
      /** \brief The memory offset of this message variable
        */
      size_t offset;
    };
    
    /** \brief Constructor (overloaded version taking a name and a data type)
      */ 
    MessageVariable(const std::string& name, const DataType& type);
    
    /** \brief Create a message variable
      */ 
    template <typename T, typename M> static MessageVariable create(const
      std::string& name, size_t offset);
  };
};

#include <variant_topic_tools/MessageVariable.tpp>

#endif
