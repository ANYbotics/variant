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

#include <ros/ros.h>

#include <variant_topic_tools/Variant.h>

namespace variant_topic_tools {
  /** \brief Message serializer/deserializer
    */  
  class MessageSerializer {
  public:
    /** \brief Default constructor
      */ 
    MessageSerializer();
    
    /** \brief Constructor for creating a message serializer by message
      *   data type
      * 
      * \note If the serializer does not exist, it will be constructed as
      *   invalid serializer.
      */ 
    MessageSerializer(const DataType& dataType);
    
    /** \brief Copy constructor
      */ 
    MessageSerializer(const MessageSerializer& src);
    
    /** \brief Destructor
      */ 
    ~MessageSerializer();
    
    /** \brief Access the data type this serializer is responsible for
      */
    DataType getDataType() const;
    
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const MessageSerializer& serializer) const {
      return (impl < serializer.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const MessageSerializer& serializer) const {
      return (impl == serializer.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const MessageSerializer& serializer) const {
      return (impl != serializer.impl);
    };
    
  protected:
    /** \brief Forward declaration of the serializer implementation type
      */
    class Impl;
    
    /** \brief Declaration of the serializer implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the serializer implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief Message serializer implementation
      */
    class Impl {
    public:
      /** \brief Default constructor
        */ 
      Impl(const DataType& dataType = DataType());
      
      /** \brief Destructor
        */ 
      virtual ~Impl();

      /** \brief Retrieve the serialized length of a variant value
        *   (abstract declaration)
        */
      virtual size_t getSerializedLength(Variant& value) const = 0;
      
      /** \brief True, if this serializer is valid
        */
      bool isValid() const;
      
      /** \brief Read a variant value from an input stream (abstract
        *   declaration)
        */
      virtual void read(ros::serialization::IStream& stream,
        Variant& value) = 0;
      
      /** \brief Write a variant value to an output stream (abstract
        *   declaration)
        */
      virtual void write(ros::serialization::OStream& stream,
        const Variant& value) = 0;
      
      /** \brief The data type this serializer is responsible for
        */
      DataType dataType;
    };
    
    /** \brief Message serializer implementation (templated version)
      */
    template <typename T> class ImplT :
      public Impl {
    public:
      /** \brief Default constructor
        */ 
      ImplT(const DataType& dataType = DataType());
      
      /** \brief Destructor
        */ 
      virtual ~ImplT();
      
      /** \brief Retrieve the serialized length of a variant value
        *   (implementation)
        */
      size_t getSerializedLength(Variant& value) const;
      
      /** \brief Read a variant value from an input stream
        *   (implementation)
        */
      void read(ros::serialization::IStream& stream, Variant& value);
      
      /** \brief Write a variant value to an output stream
        *   (implementation)
        */
      void write(ros::serialization::OStream& stream, const Variant& value);
    };
    
    /** \brief Message serializer instances
      */
    class Instances:
      public boost::unordered_map<DataType, MessageSerializer> {
    public:
      /** \brief Default constructor
        */
      Instances();
      
      /** \brief Destructor
        */
      ~Instances();
    };
    
    /** \brief Access the serializer instances
      */ 
    static Instances& getInstances();
    
    /** \brief Create a simple serializer
      */
    template <typename T> static MessageSerializer createSimple(const
      DataType& dataType);
    
    /** \brief The serializer's implementation
      */
    ImplPtr impl;
  };
};

#include <variant_topic_tools/MessageSerializer.tpp>

#endif
