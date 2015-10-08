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

/** \file MessageVariant.h
  * \brief Header file providing the MessageVariant class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_VARIANT_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_VARIANT_H

#include <vector>

#include <variant_topic_tools/MessageFieldCollection.h>
#include <variant_topic_tools/MessageMember.h>
#include <variant_topic_tools/CollectionVariant.h>

namespace variant_topic_tools {
  /** \brief Message variant type
    */  
  class MessageVariant :
    public CollectionVariant {
  friend class MessageDataType;
  friend class Variant;
  public:
    /** \brief Default constructor
      */ 
    MessageVariant();
    
    /** \brief Copy constructor
      */ 
    MessageVariant(const MessageVariant& src);
    
    /** \brief Copy constructor (overloaded version taking a variant)
      */ 
    MessageVariant(const Variant& src);
    
    /** \brief Destructor
      */ 
    ~MessageVariant();
        
    using Variant::operator=;
    
  protected:
    /** \brief Message variant value (abstract base)
      */
    class Value :
      public CollectionVariant::Value {
    public:
      /** \brief Default constructor
        */ 
      Value();
      
      /** \brief Destructor
        */ 
      virtual ~Value();
      
      /** \brief Retrieve the name of the message member with the specified
        *   index (abstract declaration)
        */
      virtual const std::string& getMemberName(size_t index) const = 0;
      
      /** \brief Write the variant collection member with the specified
        *   index to a stream (implementation)
        */
      void writeMember(std::ostream& stream, size_t index) const;
    };
    
    /** \brief Message variant value (variant-typed implementation)
      */
    class ValueImplV :
      public Value {
    public:
      /** \brief Default constructor
        */ 
      ValueImplV(const std::vector<MessageMember>& members =
        std::vector<MessageMember>());
      
      /** \brief Copy constructor
        */ 
      ValueImplV(const ValueImplV& src);
      
      /** \brief Destructor
        */ 
      virtual ~ValueImplV();
      
      /** \brief Retrieve the number of members of the variant collection
        *   (implementation)
        */
      size_t getNumMembers() const;
      
      /** \brief Set a member of the variant collection by index
        *   (implementation)
        */
      void setMember(size_t index, const Variant& member);
      
      /** \brief Set a member of the variant collection by name
        *   (implementation)
        */
      void setMember(const std::string& name, const Variant& member);
    
      /** \brief Retrieve a member of the variant collection by index
        *   (implementation)
        */
      Variant getMember(size_t index) const;
      
      /** \brief Retrieve a member of the variant collection by name
        *   (implementation)
        */
      Variant getMember(const std::string& name) const;
      
      /** \brief Retrieve the name of the message member with the specified
        *   index (implementation)
        */
      const std::string& getMemberName(size_t index) const;
      
      /** \brief True, if the variant collection contains the member with the
        *   specified name (implementation)
        */
      bool hasMember(const std::string& name) const;
      
      /** \brief Clone this variant value (implementation)
        */
      ValuePtr clone() const;
      
      /** \brief The message members
        */
      MessageFieldCollection<Variant> members;
    };
    
    /** \brief Message variant value (templated implementation)
      */
    template <typename T> class ValueImplT :
      public Variant::ValueT<T>,
      public Value {
    public:
      /** \brief Default constructor
        */ 
      ValueImplT(const std::vector<MessageMember>&
        members = std::vector<MessageMember>(), const Pointer<T>&
        message = Pointer<T>());
      
      /** \brief Copy constructor
        */ 
      ValueImplT(const ValueImplT<T>& src);
      
      /** \brief Destructor
        */ 
      virtual ~ValueImplT();
      
      /** \brief Set the variant's value pointer (implementation)
        */
      void set(const Pointer<T>& value);
      
      /** \brief Set the variant's value (implementation)
        */
      void setValue(const T& value);
      
      /** \brief Retrieve the variant's value (implementation of the
        *   non-const version)
        */
      T& getValue();
      
      /** \brief Retrieve the variant's value (implementation of the
        *   const version)
        */
      const T& getValue() const;
      
      /** \brief Retrieve the number of members of the variant collection
        *   (implementation)
        */
      size_t getNumMembers() const;
      
      /** \brief Set a member of the variant collection by index
        *   (implementation)
        */
      void setMember(size_t index, const Variant& member);
      
      /** \brief Set a member of the variant collection by name
        *   (implementation)
        */
      void setMember(const std::string& name, const Variant& member);
    
      /** \brief Retrieve a member of the variant collection by index
        *   (implementation)
        */
      Variant getMember(size_t index) const;
      
      /** \brief Retrieve a member of the variant collection by name
        *   (implementation)
        */
      Variant getMember(const std::string& name) const;
      
      /** \brief Retrieve the name of the message member with the specified
        *   index (implementation)
        */
      const std::string& getMemberName(size_t index) const;
      
      /** \brief True, if the variant collection contains the member with the
        *   specified name (implementation)
        */
      bool hasMember(const std::string& name) const;
      
      /** \brief True, if this variant value equals another variant value
        *   (re-implementation)
        */
      bool isEqual(const Variant::Value& value) const;
      
      /** \brief Clone this variant value (implementation)
        */
      ValuePtr clone() const;
      
      /** \brief Read the variant from a stream (re-implementation)
        */
      void read(std::istream& stream);
    
      /** \brief Write this variant value to a stream (re-implementation)
        */
      void write(std::ostream& stream) const;
      
      /** \brief The message members
        */
      MessageFieldCollection<MessageMember> members;
      
      /** \brief The strong-typed message
        */
      mutable Pointer<T> message;
    };
    
    /** \brief Constructor (overloaded version taking a message data type
      *   and a message field collection)
      */ 
    MessageVariant(const DataType& type, const std::vector<MessageMember>&
      members);
    
    /** \brief Create a message variant
      */ 
    template <typename T> static MessageVariant create(const DataType& type,
      const std::vector<MessageMember>& members);
  };
};

#include <variant_topic_tools/MessageVariant.tpp>

#endif
