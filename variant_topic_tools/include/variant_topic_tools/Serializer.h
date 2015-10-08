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

/** \file Serializer.h
  * \brief Header file providing the Serializer class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_SERIALIZER_H
#define VARIANT_TOPIC_TOOLS_SERIALIZER_H

#include <ros/ros.h>

#include <variant_topic_tools/Forwards.h>

namespace variant_topic_tools {
  /** \brief Data type
    */
  class Serializer {
  public:
    /** \brief Default constructor
      */ 
    Serializer();
    
    /** \brief Constructor (overloaded version taking a data type)
      * 
      * \note If the data type specified data type cannot be found in the
      *   data type registry, the instantiated serializer will be invalid.
      */ 
    Serializer(const DataType& dataType);
    
    /** \brief Copy constructor
      */ 
    Serializer(const Serializer& src);
    
    /** \brief Destructor
      */ 
    ~Serializer();

    /** \brief True, if this serializer is valid
      */ 
    bool isValid() const;
    
    /** \brief Clear the serializer
      */
    void clear();
    
    /** \brief Serialize a variant value to an output stream
      */ 
    void serialize(ros::serialization::OStream& stream, const Variant& value);
    
    /** \brief Deserialize a variant value from an input stream
      */ 
    void deserialize(ros::serialization::IStream& stream, Variant& value);
    
    /** \brief Advance an input stream by the length of a serialized value
      */ 
    void advance(ros::serialization::IStream& stream);
    
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl) ? (void*)1 : (void*)0;
    };
    
  protected:
    /** \brief Serializer implementation
      */
    class Impl {
    public:
      /** \brief Default constructor
        */
      Impl();
      
      /** \brief Destructor
        */
      virtual ~Impl();

      /** \brief Serialize a variant value (abstract declaration)
        */ 
      virtual void serialize(ros::serialization::OStream& stream, const
        Variant& value) = 0;
      
      /** \brief Deserialize a variant value (abstract declaration)
        */ 
      virtual void deserialize(ros::serialization::IStream& stream,
        Variant& value) = 0;
        
      /** \brief Advance an input stream by the length of a serialized
        *   value (abstract declaration)
        */ 
      virtual void advance(ros::serialization::IStream& stream) = 0;
    };
    
    /** \brief Declaration of the serializer implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the serializer implementation weak pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The serializer's implementation
      */
    ImplPtr impl;
    
    /** \brief Advance an input stream by the length of a serialized
      *   value (overloaded version taking a fixed-size message)
      */ 
    template <typename T> static void advance(ros::serialization::IStream&
      stream, typename boost::enable_if<ros::message_traits::IsFixedSize<
      T> >::type* = 0);
    
    /** \brief Advance an input stream by the length of a serialized
      *   value (overloaded version taking a non-fixed-size message)
      */ 
    template <typename T> static void advance(ros::serialization::IStream&
      stream, typename boost::disable_if<ros::message_traits::IsFixedSize<
      T> >::type* = 0);
  };
};

#include <variant_topic_tools/Serializer.tpp>

#endif
