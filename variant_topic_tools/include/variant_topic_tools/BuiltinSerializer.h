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

/** \file BuiltinSerializer.h
  * \brief Header file providing the BuiltinSerializer class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_BUILTIN_SERIALIZER_H
#define VARIANT_TOPIC_TOOLS_BUILTIN_SERIALIZER_H

#include <variant_topic_tools/Serializer.h>

namespace variant_topic_tools {
  /** \brief Built-in serializer
    */
  class BuiltinSerializer :
    public Serializer {
  friend class BuiltinDataType;
  public:
    /** \brief Default constructor
      */ 
    BuiltinSerializer();
    
    /** \brief Copy constructor
      */ 
    BuiltinSerializer(const BuiltinSerializer& src);
    
    /** \brief Copy constructor (overloaded version taking a serializer)
      */ 
    BuiltinSerializer(const Serializer& src);
    
    /** \brief Destructor
      */ 
    ~BuiltinSerializer();
    
  protected:
    /** \brief Built-in data serializer implementation
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
    
    /** \brief Built-in serializer implementation (templated strong-typed
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
    
    /** \brief Create a built-in serializer
      */ 
    template <typename T> static BuiltinSerializer create();    
  };
};

#include <variant_topic_tools/BuiltinSerializer.tpp>

#endif
