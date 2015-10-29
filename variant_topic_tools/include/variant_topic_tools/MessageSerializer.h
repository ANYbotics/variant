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

/** \file MessageSerializer.h
  * \brief Header file providing the MessageSerializer class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_SERIALIZER_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_SERIALIZER_H

#include <variant_topic_tools/MessageFieldCollection.h>
#include <variant_topic_tools/MessageTypeTraits.h>
#include <variant_topic_tools/Serializer.h>

namespace variant_topic_tools {
  /** \brief Message serializer
    */
  class MessageSerializer :
    public Serializer {
  friend class MessageDataType;
  friend class MessageVariant;
  public:
    /** \brief Default constructor
      */ 
    MessageSerializer();
    
    /** \brief Copy constructor
      */ 
    MessageSerializer(const MessageSerializer& src);
    
    /** \brief Copy constructor (overloaded version taking a serializer)
      */ 
    MessageSerializer(const Serializer& src);
    
    /** \brief Destructor
      */ 
    ~MessageSerializer();
    
  protected:
    /** \brief Message serializer implementation
      */
    class Impl :
      public virtual Serializer::Impl {
    public:
      /** \brief Default constructor
        */
      Impl();
      
      /** \brief Destructor
        */
      virtual ~Impl();
    };
    
    /** \brief Message serializer implementation (variant-typed version)
      */
    class ImplV :
      public Impl {
    public:
      /** \brief Constructor
        */
      ImplV(const MessageFieldCollection<Serializer>& memberSerializers =
        MessageFieldCollection<Serializer>());
      
      /** \brief Destructor
        */
      virtual ~ImplV();
    
      /** \brief Serialize a variant value (implementation)
        */ 
      void serialize(ros::serialization::OStream& stream, const
        Variant& value);
      
      /** \brief Deserialize a variant value (implementation)
        */ 
      void deserialize(ros::serialization::IStream& stream, Variant& value);
        
      /** \brief Advance an input stream by the length of a serialized
        *   value (implementation)
        */ 
      void advance(ros::serialization::IStream& stream, const Variant& value);
      
      /** \brief The message member serializers
        */
      MessageFieldCollection<Serializer> memberSerializers;
    };
    
    /** \brief Message serializer implementation (templated strong-typed
      *   version)
      */
    template <typename T> class ImplT :
      public Impl {
    public:
      BOOST_STATIC_ASSERT(type_traits::IsMessage<T>::value);
      
      /** \brief Definition of the value type
        */
      typedef typename type_traits::MessageType<T>::ValueType ValueType;
      
      /** \brief Default constructor
        */
      ImplT();
      
      /** \brief Destructor
        */
      virtual ~ImplT();
    
      /** \brief Serialize a variant value (implementation)
        */ 
      void serialize(ros::serialization::OStream& stream, const
        Variant& value);
      
      /** \brief Deserialize a variant value (implementation)
        */ 
      void deserialize(ros::serialization::IStream& stream, Variant& value);
        
      /** \brief Advance an input stream by the length of a serialized
        *   value (implementation)
        */ 
      void advance(ros::serialization::IStream& stream, const Variant& value);
    };
    
    /** \brief Constructor (overloaded version taking a collection of member
      *   serializers)
      */ 
    MessageSerializer(const MessageFieldCollection<Serializer>&
      memberSerializers);
    
    
    /** \brief Create a message serializer
      */ 
    template <typename T> static MessageSerializer create();
  };
};

#include <variant_topic_tools/MessageSerializer.tpp>

#endif
