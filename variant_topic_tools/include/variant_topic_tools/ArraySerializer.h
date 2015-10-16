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

/** \file ArraySerializer.h
  * \brief Header file providing the ArraySerializer class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_ARRAY_SERIALIZER_H
#define VARIANT_TOPIC_TOOLS_ARRAY_SERIALIZER_H

#include <variant_topic_tools/ArrayTypeTraits.h>
#include <variant_topic_tools/Serializer.h>

namespace variant_topic_tools {
  /** \brief Array serializer
    */
  class ArraySerializer :
    public Serializer {
  friend class ArrayDataType;
  public:
    /** \brief Default constructor
      */ 
    ArraySerializer();
    
    /** \brief Copy constructor
      */ 
    ArraySerializer(const ArraySerializer& src);
    
    /** \brief Copy constructor (overloaded version taking a serializer)
      */ 
    ArraySerializer(const Serializer& src);
    
    /** \brief Destructor
      */ 
    ~ArraySerializer();
    
  protected:
    /** \brief Array serializer implementation
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
    
    /** \brief Array serializer implementation (variant-typed version)
      */
    class ImplV :
      public Impl {
    public:
      /** \brief Constructor
        */
      ImplV(const Serializer& memberSerializer, size_t numMembers);
      
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
      void advance(ros::serialization::IStream& stream);
    };
    
    /** \brief Array serializer implementation (templated strong-typed
      *   version)
      */
    template <typename T> class ImplT :
      public Impl {
    public:
      BOOST_STATIC_ASSERT(type_traits::IsArray<T>::value);
      
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
      void advance(ros::serialization::IStream& stream);
    };
    
    /** \brief Constructor (overloaded version taking an member serializer
      *   and a number of members)
      */ 
    ArraySerializer(const Serializer& memberSerializer, size_t numMembers);
    
    /** \brief Create an array serializer
      */ 
    template <typename T> static ArraySerializer create();
  };
};

#include <variant_topic_tools/ArraySerializer.tpp>

#endif
