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

/** \file MessageDataType.h
  * \brief Header file providing the MessageDataType class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_DATA_TYPE_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_DATA_TYPE_H

#include <variant_topic_tools/DataType.h>
#include <variant_topic_tools/MessageConstant.h>
#include <variant_topic_tools/MessageMember.h>
#include <variant_topic_tools/MessageVariable.h>

namespace variant_topic_tools {
  /** \brief Message data type
    */
  class MessageDataType :
    public DataType {
  friend class DataType;
  friend class DataTypeRegistry;
  friend class VariantMessage;
  public:
    /** \brief Default constructor
      */ 
    MessageDataType();
    
    /** \brief Copy constructor
      */ 
    MessageDataType(const MessageDataType& src);
    
    /** \brief Copy constructor (overloaded version taking a data type)
      */ 
    MessageDataType(const DataType& src);
    
    /** \brief Destructor
      */ 
    virtual ~MessageDataType();
  
    /** \brief Retrieve the MD5 sum of this message data type
      */
    const std::string& getMD5Sum() const;
    
    /** \brief Retrieve the definition this message data type
      */
    const std::string& getDefinition() const;
    
    /** \brief Retrieve the number of members of this message data type
      */
    size_t getNumMembers() const;
    
    /** \brief Access a member of the message data type by index
      */
    const MessageMember& getMember(size_t index) const;
    
    /** \brief True, if this message data type is simple
      */
    bool isSimple() const;
    
    /** \brief Add a constant member to this message data type (version
      *   taking a variant value)
      */
    MessageConstant addConstant(const std::string& name, const Variant& value);
    
    /** \brief Add a constant member to this message data type (version
      *   templated on the value type)
      */
    template <typename T> MessageConstant addConstant(const std::string& name,
      const T& value);
    
    /** \brief Add a variable member to this message data type (version
      *   taking a data type)
      */
    MessageVariable addVariable(const std::string& name, const DataType& type);
    
    /** \brief Add a variable member to this message data type (version
      *   templated on the variable type)
      */
    template <typename T> MessageVariable addVariable(const std::string& name);
    
    /** \brief Add a member to this message data type
      */
    void addMember(const MessageMember& member);
    
    /** \brief Assignment operator
      */
    MessageDataType& operator=(const DataType& src);
    
    /** \brief Operator for accessing the members of the message data type
      *   by index
      */
    const MessageMember& operator[](size_t index) const;
    
  protected:
    /** \brief Type traits
      */
    struct TypeTraits {
      template <typename T> struct IsMessage :
        public ros::message_traits::IsMessage<T> {
      };
    };
    
    /** \brief Message data type implementation
      */
    class Impl :
      public DataType::Impl {
    public:
      /** \brief Constructor (overloaded version taking a sequence of
        *   members)
        */
      Impl(const std::vector<MessageMember>& members);
      
      /** \brief Constructor (overloaded version taking a definition)
        */
      Impl(const std::string& definition);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the MD5 sum of this message data type (abstract
        *   declaration)
        */
      virtual const std::string& getMD5Sum() const = 0;
      
      /** \brief Retrieve the definition this message data type (abstract
        *   declaration)
        */
      virtual const std::string& getDefinition() const = 0;
      
      /** \brief True, if this message data type is simple (abstract
        *   declaration)
        */
      virtual bool isSimple() const = 0;
      
      /** \brief Add a member to this message data type (abstract declaration)
        */
      virtual void addMember(const MessageMember& member) = 0;
    
      /** \brief The members of this message data type
        */
      std::vector<MessageMember> members;
    };
    
    /** \brief Message data type implementation (variant-typed version)
      */
    class ImplV :
      public Impl {
    public:
      /** \brief Constructor (overloaded version taking an identifier and
        *   a sequence of members)
        */
      ImplV(const std::string& identifier, const std::vector<MessageMember>&
        members);
      
      /** \brief Constructor (overloaded version taking an identifier and
        *   a definition)
        */
      ImplV(const std::string& identifier, const std::string& definition);
      
      /** \brief Destructor
        */
      virtual ~ImplV();
      
      /** \brief Retrieve the identifier representing this data type
        *   (implementation)
        */ 
      const std::string& getIdentifier() const;
      
      /** \brief Retrieve the size of the instances of this data type
        *   (implementation)
        */
      size_t getSize() const;
      
      /** \brief Retrieve the MD5 sum of this message data type
        *   (implementation)
        */
      const std::string& getMD5Sum() const;
      
      /** \brief Retrieve the definition this message data type
        *   (implementation)
        */
      const std::string& getDefinition() const;
      
      /** \brief True, if this message data type is simple (implementation)
        */
      bool isSimple() const;
      
      /** \brief True, if this message data type has fixed size
        *   (implementation)
        */
      bool isFixedSize() const;
      
      /** \brief Create a serializer for this data type (re-implementation)
        */ 
      Serializer createSerializer() const;
      
      /** \brief Create a variant from this data type (re-implementation)
        */ 
      Variant createVariant() const;
      
      /** \brief Add a member to this message data type (implementation)
        */
      void addMember(const MessageMember& member);
      
      /** \brief The identifier representing this message data type
        */
      std::string identifier;
      
      /** \brief The definition of this message data type
        */
      std::string definition;
    };
    
    /** \brief Message data type implementation (templated strong-typed
      *   version)
      */
    template <typename T> class ImplT :
      public Impl {
    public:
      /** \brief Default constructor
        */
      ImplT();
      
      /** \brief Destructor
        */
      virtual ~ImplT();
      
      /** \brief Retrieve the identifier representing this data type
        *   (implementation)
        */ 
      const std::string& getIdentifier() const;
      
      /** \brief Retrieve the type information associated with this data type
        *   (re-implementation)
        */ 
      const std::type_info& getTypeInfo() const;
      
      /** \brief Retrieve the size of the instances of this data type
        *   (implementation)
        */
      size_t getSize() const;
      
      /** \brief Retrieve the MD5 sum of this message data type
        *   (implementation)
        */
      const std::string& getMD5Sum() const;
      
      /** \brief Retrieve the definition this message data type
        *   (implementation)
        */
      const std::string& getDefinition() const;
      
      /** \brief True, if this data type represents a fixed-size data type,
        *   as opposed to a variable-size data type (implementation)
        */ 
      bool isFixedSize() const;
      
      /** \brief True, if this message data type is simple (implementation)
        */
      bool isSimple() const;
      
      /** \brief Create a serializer for this data type (re-implementation)
        */ 
      Serializer createSerializer() const;
      
      /** \brief Create a variant from this data type (re-implementation)
        */ 
      Variant createVariant() const;
      
      /** \brief Add a member to this message data type (implementation)
        */
      void addMember(const MessageMember& member);
    };
    
    /** \brief Constructor (overloaded version taking an identifier and a
      *   sequence of message members)
      */ 
    MessageDataType(const std::string& identifier, const
      std::vector<MessageMember>& members);
    
    /** \brief Constructor (overloaded version taking an identifier and
      *   a definition)
      */ 
    MessageDataType(const std::string& identifier, const std::string&
      definition);
    
    /** \brief Create a message data type (version templated on the
      *   message type)
      */ 
    template <typename T> static MessageDataType create();
  };
};

#include <variant_topic_tools/MessageDataType.tpp>

#endif
