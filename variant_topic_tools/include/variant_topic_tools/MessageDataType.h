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
#include <variant_topic_tools/DataTypeTraits.h>
#include <variant_topic_tools/MD5Sum.h>
#include <variant_topic_tools/MessageFieldCollection.h>
#include <variant_topic_tools/MessageTypeTraits.h>

namespace variant_topic_tools {
  /** \brief Message data type
    */
  class MessageDataType :
    public DataType {
  friend class DataType;
  friend class DataTypeRegistry;
  friend class MessageVariant;
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
    std::string getMD5Sum() const;
    
    /** \brief Retrieve the definition this message data type
      */
    const std::string& getDefinition() const;
    
    /** \brief Retrieve the number of members of this message data type
      */
    size_t getNumMembers() const;
    
    /** \brief Retrieve the number of constant members of this message data
      *   type
      */
    size_t getNumConstantMembers() const;
    
    /** \brief Retrieve the number of variable members of this message data
      *   type
      */
    size_t getNumVariableMembers() const;
    
    /** \brief Access a member of the message data type by name
      */
    const MessageMember& getMember(const std::string& name) const;
    
    /** \brief Access a member of the message data type by index
      */
    const MessageMember& getMember(size_t index) const;
    
    /** \brief Access a constant member of the message data type by name
      */
    const MessageConstant& getConstantMember(const std::string& name) const;
    
    /** \brief Access a constant member of the message data type by index
      */
    const MessageConstant& getConstantMember(size_t index) const;
    
    /** \brief Access a variable member of the message data type by name
      */
    const MessageVariable& getVariableMember(const std::string& name) const;
    
    /** \brief Access a variable member of the message data type by index
      */
    const MessageVariable& getVariableMember(size_t index) const;
    
    /** \brief True, if the message data type contains the member with the
      *   specified name
      */
    bool hasMember(const std::string& name) const;
    
    /** \brief True, if the message data type contains the constant member
      *   with the specified name
      */
    bool hasConstantMember(const std::string& name) const;
    
    /** \brief True, if the message data type contains the variable member
      *   with the specified name
      */
    bool hasVariableMember(const std::string& name) const;
    
    /** \brief True, if this message data type has a header
      */
    bool hasHeader() const;
    
    /** \brief Add a constant member to this message data type (overloaded
      *   version taking a message constant)
      */
    void addConstantMember(const MessageConstant& member);
    
    /** \brief Add a constant member to this message data type (overloaded
      *   version taking a name and a variant value)
      */
    MessageConstant addConstantMember(const std::string& name, const
      Variant& value);
    
    /** \brief Add a constant member to this message data type (version
      *   templated on the value type)
      */
    template <typename T> MessageConstant addConstantMember(const
      std::string& name, const T& value);
    
    /** \brief Add a variable member to this message data type (overloaded
      *   version taking a message variable)
      */
    void addVariableMember(const MessageVariable& member);
    
    /** \brief Add a variable member to this message data type (overloaded
      *   version taking a name and a data type)
      */
    MessageVariable addVariableMember(const std::string& name, const
      DataType& type);
    
    /** \brief Add a variable member to this message data type (version
      *   templated on the variable type)
      */
    template <typename T> MessageVariable addVariableMember(const
      std::string& name);
    
    /** \brief Assignment operator
      */
    MessageDataType& operator=(const DataType& src);
    
    /** \brief Operator for accessing the members of the message data type
      *   by index
      */
    const MessageMember& operator[](size_t index) const;
    
  protected:
    /** \brief Message data type implementation
      */
    class Impl :
      public DataType::Impl {
    public:
      /** \brief Constructor (overloaded version taking a collection of
        *   constant members and a collection of variable members)
        */
      Impl(const MessageFieldCollection<MessageConstant>& constantMembers,
        const MessageFieldCollection<MessageVariable>& variableMembers);
      
      /** \brief Constructor (overloaded version taking a definition)
        */
      Impl(const std::string& identifier, const std::string& definition);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the MD5 sum of this message data type (abstract
        *   declaration)
        */
      virtual std::string getMD5Sum() const = 0;
      
      /** \brief Retrieve the definition this message data type (abstract
        *   declaration)
        */
      virtual const std::string& getDefinition() const = 0;
      
      /** \brief Add a constant member to this message data type (abstract
        *   declaration)
        */
      virtual void addConstantMember(const MessageConstant& member) = 0;
    
      /** \brief Add a variable member to this message data type (abstract
        *   declaration)
        */
      virtual void addVariableMember(const MessageVariable& member) = 0;
    
      /** \brief The constant members of this message data type
        */
      MessageFieldCollection<MessageConstant> constantMembers;
      
      /** \brief The variable members of this message data type
        */
      MessageFieldCollection<MessageVariable> variableMembers;
    };
    
    /** \brief Message data type implementation (variant-typed version)
      */
    class ImplV :
      public Impl {
    public:
      /** \brief Constructor (overloaded version taking an identifier, a
        *   sequence of constant members, and a sequence of variable members)
        */
      ImplV(const std::string& identifier, const MessageFieldCollection<
        MessageConstant>& constantMembers, const MessageFieldCollection<
        MessageVariable>& variableMembers);
      
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
      
      /** \brief Retrieve the MD5 sum of this message data type
        *   (implementation)
        */
      std::string getMD5Sum() const;
      
      /** \brief Retrieve the definition this message data type
        *   (implementation)
        */
      const std::string& getDefinition() const;
      
      /** \brief Retrieve the size of the instances of this data type
        *   (implementation)
        */
      size_t getSize() const;
      
      /** \brief True, if this data type represents a fixed-size data type
        *   (implementation)
        */ 
      bool isFixedSize() const;
      
      /** \brief True, if this data type represents a simple data type
        *   (implementation)
        */ 
      bool isSimple() const;
      
      /** \brief Create a serializer for this data type (re-implementation)
        */ 
      Serializer createSerializer(const DataType& type) const;
      
      /** \brief Create a variant from this data type (re-implementation)
        */ 
      Variant createVariant(const DataType& type) const;
      
      /** \brief Add a constant member to this message data type
        *   (implementation)
        */
      void addConstantMember(const MessageConstant& member);
      
      /** \brief Add a variable member to this message data type
        *   (implementation)
        */
      void addVariableMember(const MessageVariable& member);
      
      /** \brief Re-calculate the MD5 sum of this message data type
        */
      void recalculateMD5Sum();
      
      /** \brief The identifier representing this message data type
        */
      std::string identifier;
      
      /** \brief The MD5 sum of this message data type
        */
      MD5Sum md5Sum;
      
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
      BOOST_STATIC_ASSERT(type_traits::IsMessage<T>::value);
  
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
      
      /** \brief Retrieve the MD5 sum of this message data type
        *   (implementation)
        */
      std::string getMD5Sum() const;
      
      /** \brief Retrieve the definition this message data type
        *   (implementation)
        */
      const std::string& getDefinition() const;
      
      /** \brief Retrieve the size of the instances of this data type
        *   (implementation)
        */
      size_t getSize() const;
      
      /** \brief True, if this data type represents a fixed-size data type
        *   (implementation)
        */ 
      bool isFixedSize() const;
      
      /** \brief True, if this data type represents a simple data type
        *   (implementation)
        */ 
      bool isSimple() const;
      
      /** \brief Create a serializer for this data type (re-implementation)
        */ 
      Serializer createSerializer(const DataType& type) const;
      
      /** \brief Create a variant from this data type (re-implementation)
        */ 
      Variant createVariant(const DataType& type) const;
      
      /** \brief Add a constant member to this message data type
        *   (implementation)
        */
      void addConstantMember(const MessageConstant& member);
      
      /** \brief Add a variable member to this message data type
        *   (implementation)
        */
      void addVariableMember(const MessageVariable& member);
      
      /** \brief Process the next variable member to determine its memory
        *   offset
        */
      template <typename M> void next(const M& member);
      
      /** \brief The message used to determine the variable member memory
        *   offsets
        */
      T message;
      
      /** \brief The index of the currently processed variable member
        */
      size_t memberIndex;
    };
    
    /** \brief Constructor (overloaded version taking an identifier, a
      *   sequence of constant members, and a sequence of variable members)
      */ 
    MessageDataType(const std::string& identifier, const
      MessageFieldCollection<MessageConstant>& constantMembers, const
      MessageFieldCollection<MessageVariable>& variableMembers);
    
    /** \brief Constructor (overloaded version taking an identifier and
      *   a definition)
      */ 
    MessageDataType(const std::string& identifier, const std::string&
      definition);
    
    /** \brief Create a message data type (version templated on the
      *   message type)
      */ 
    template <typename T> static MessageDataType create();
    
    /** \brief Add a variable message member (overloaded version for adding
      *   a dynamic array member)
      */ 
    template <typename T, typename M> static void addMember(
      MessageVariable& member, size_t offset, typename boost::enable_if<
      type_traits::IsArray<M> >::type* = 0, typename boost::enable_if<
      typename type_traits::ArrayType<M>::IsDynamic>::type* = 0);
    
    /** \brief Add a variable message member (overloaded version for adding
      *   a non-dynamic array member)
      */ 
    template <typename T, typename M> static void addMember(
      MessageVariable& member, size_t offset, typename boost::enable_if<
      type_traits::IsArray<M> >::type* = 0, typename boost::disable_if<
      typename type_traits::ArrayType<M>::IsDynamic>::type* = 0);
    
    /** \brief Add a variable message member (overloaded version for adding
      *   a non-array member)
      */ 
    template <typename T, typename M> static void addMember(
      MessageVariable& member, size_t offset, typename boost::
      disable_if<type_traits::IsArray<M> >::type* = 0);
  };
};

#include <variant_topic_tools/MessageDataType.tpp>

#endif
