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

#include <variant_topic_tools/CollectionVariant.h>
#include <variant_topic_tools/MessageFieldCollection.h>
#include <variant_topic_tools/MessageTypeTraits.h>

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

      /** \brief Set the variant's value (implementation)
        */
      void setValue(const Variant::Value& value);

      /** \brief Retrieve the name of the message member with the specified
        *   index (abstract declaration)
        */
      virtual const std::string& getMemberName(int index) const = 0;

      /** \brief Write the variant collection member with the specified
        *   index to a stream (implementation)
        */
      void writeMember(std::ostream& stream, int index) const;
    };

    /** \brief Message variant value (variant-typed implementation)
      */
    class ValueImplV :
      public Value {
    public:
      /** \brief Default constructor
        */
      ValueImplV(const MessageFieldCollection<Variant>& members =
        MessageFieldCollection<Variant>());

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
      void setMember(int index, const Variant& member);

      /** \brief Set a member of the variant collection by name
        *   (implementation)
        */
      void setMember(const std::string& name, const Variant& member);

      /** \brief Retrieve a member of the variant collection by index
        *   (implementation)
        */
      Variant getMember(int index) const;

      /** \brief Retrieve a member of the variant collection by name
        *   (implementation)
        */
      Variant getMember(const std::string& name) const;

      /** \brief Retrieve the name of the message member with the specified
        *   index (implementation)
        */
      const std::string& getMemberName(int index) const;

      /** \brief True, if the variant collection contains the member with the
        *   specified name (implementation)
        */
      bool hasMember(const std::string& name) const;

      /** \brief Clone this variant value (implementation)
        */
      ValuePtr clone() const;

      /** \brief Create a serializer for this variant (implementation)
        */
      Serializer createSerializer(const DataType& type) const;

      /** \brief The message members
        */
      MessageFieldCollection<Variant> members;
    };

    /** \brief Message variant value (templated implementation)
      */
    template <typename T> class ValueImplT :
      public Variant::ValueT<typename type_traits::MessageType<T>::ValueType>,
      public Value {
    public:
      BOOST_STATIC_ASSERT(type_traits::IsMessage<T>::value);

      /** \brief Definition of the value type
        */
      typedef typename type_traits::MessageType<T>::ValueType ValueType;

      /** \brief Default constructor
        */
      ValueImplT(const MessageFieldCollection<MessageVariable>& members =
        MessageFieldCollection<MessageVariable>(), const Pointer<ValueType>&
        message = Pointer<ValueType>());

      /** \brief Copy constructor
        */
      ValueImplT(const ValueImplT<T>& src);

      /** \brief Destructor
        */
      virtual ~ValueImplT();

      /** \brief Set the variant's value pointer (implementation)
        */
      void set(const Pointer<ValueType>& value);

      /** \brief Retrieve the variant's value (implementation of the
        *   non-const version)
        */
      ValueType& getValue();

      /** \brief Retrieve the variant's value (implementation of the
        *   const version)
        */
      const ValueType& getValue() const;

      /** \brief Retrieve the number of members of the variant collection
        *   (implementation)
        */
      size_t getNumMembers() const;

      /** \brief Set a member of the variant collection by index
        *   (implementation)
        */
      void setMember(int index, const Variant& member);

      /** \brief Set a member of the variant collection by name
        *   (implementation)
        */
      void setMember(const std::string& name, const Variant& member);

      /** \brief Retrieve a member of the variant collection by index
        *   (implementation)
        */
      Variant getMember(int index) const;

      /** \brief Retrieve a member of the variant collection by name
        *   (implementation)
        */
      Variant getMember(const std::string& name) const;

      /** \brief Retrieve the name of the message member with the specified
        *   index (implementation)
        */
      const std::string& getMemberName(int index) const;

      /** \brief True, if the variant collection contains the member with the
        *   specified name (implementation)
        */
      bool hasMember(const std::string& name) const;

      /** \brief Clone this variant value (implementation)
        */
      ValuePtr clone() const;

      /** \brief Create a serializer for this variant (implementation)
        */
      Serializer createSerializer(const DataType& type) const;

      /** \brief The message members
        */
      MessageFieldCollection<MessageVariable> members;

      /** \brief The strong-typed message
        */
      mutable Pointer<ValueType> message;
    };

    /** \brief Constructor (overloaded version taking a message data type
      *   and a collection of member variants)
      */
    MessageVariant(const DataType& type, const MessageFieldCollection<
      Variant>& members);

    /** \brief Create a message variant
      */
    template <typename T> static MessageVariant create(const DataType& type,
      const MessageFieldCollection<MessageVariable>& members);
  };
};

#include <variant_topic_tools/MessageVariant.tpp>

#endif
