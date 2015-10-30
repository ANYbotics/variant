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

/** \file BuiltinVariant.h
  * \brief Header file providing the BuiltinVariant class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_BUILTIN_VARIANT_H
#define VARIANT_TOPIC_TOOLS_BUILTIN_VARIANT_H

#include <boost/type_traits.hpp>

#include <variant_topic_tools/BuiltinTypeTraits.h>
#include <variant_topic_tools/Pointer.h>
#include <variant_topic_tools/Variant.h>

namespace variant_topic_tools {
  /** \brief Built-in variant type
    */  
  class BuiltinVariant :
    public Variant {
  friend class BuiltinDataType;
  friend class Variant;
  public:
    /** \brief Default constructor
      */ 
    BuiltinVariant();
    
    /** \brief Copy constructor
      */ 
    BuiltinVariant(const BuiltinVariant& src);
    
    /** \brief Copy constructor (overloaded version taking a variant)
      */ 
    BuiltinVariant(const Variant& src);
    
    /** \brief Destructor
      */ 
    ~BuiltinVariant();

    /** \brief Retrieve the built-in variant's numeric value
      */
    double getNumericValue() const;
      
    using Variant::operator=;
    
  protected:
    /** \brief Built-in variant value (abstract base)
      */
    class Value :
      public virtual Variant::Value {
    public:
      /** \brief Default constructor
        */ 
      Value();
      
      /** \brief Destructor
        */ 
      virtual ~Value();
      
      /** \brief Retrieve the built-in variant's numeric value (abstract
        *   declaration)
        */
      virtual double getNumericValue() const = 0;
    };
    
    /** \brief Built-in variant value (templated implementation)
      */
    template <typename T> class ValueImplT :
      public Variant::ValueT<typename type_traits::BuiltinType<T>::ValueType>,
      public Value {
    public:
      BOOST_STATIC_ASSERT(type_traits::IsBuiltin<T>::value);
      
      /** \brief Declaration of the value type
        */
      typedef typename type_traits::BuiltinType<T>::ValueType ValueType;
      
      /** \brief Default constructor
        */ 
      ValueImplT(const Pointer<ValueType>& value = Pointer<ValueType>());
      
      /** \brief Copy constructor
        */ 
      ValueImplT(const ValueImplT<T>& src);
      
      /** \brief Destructor
        */ 
      virtual ~ValueImplT();
      
      /** \brief Set the variant's value pointer (implementation)
        */
      void set(const Pointer<ValueType>& value);
      
      /** \brief Set the variant's value (implementation)
        */
      void setValue(const Variant::Value& value);
      
      /** \brief Retrieve the variant's value (implementation of the
        *   non-const version)
        */
      ValueType& getValue();
      
      /** \brief Retrieve the variant's value (implementation of the
        *   const version)
        */
      const ValueType& getValue() const;
      
      /** \brief Retrieve the built-in variant's numeric value
        *   (implementation)
        */
      double getNumericValue() const;
    
      /** \brief True, if this variant value equals another variant value
        *   (implementation)
        */
      bool isEqual(const Variant::Value& value) const;
      
      /** \brief Clone this variant value (implementation)
        */
      ValuePtr clone() const;
      
      /** \brief Read the variant from a stream (implementation)
        */
      void read(std::istream& stream);
    
      /** \brief Write this variant value to a stream (implementation)
        */
      void write(std::ostream& stream) const;
      
      /** \brief Create a serializer for this variant (implementation)
        */ 
      Serializer createSerializer(const DataType& type) const;
      
      /** \brief The strong-typed value
        */
      Pointer<ValueType> value;
    };
    
    /** \brief Create a built-in variant
      */ 
    template <typename T> static BuiltinVariant create(const DataType& type);
    
    /** \brief Retrieve the built-in variant's numeric value (overloaded
      *   version taking a ROS duration or ROS time)
      */
    template <typename T> static double getNumericValue(const T& value,
      typename boost::enable_if<boost::type_traits::ice_or<boost::is_base_of<
      ros::Duration, T>::value, boost::is_base_of<ros::Time, T>::value> >::
      type* = 0);
      
    /** \brief Retrieve the built-in variant's numeric value (overloaded
      *   version taking a numeric value other than a ROS duration or a
      *   ROS time)
      */
    template <typename T> static double getNumericValue(const T& value,
      typename boost::enable_if<typename type_traits::BuiltinType<T>::
      IsNumeric>::type* = 0, typename boost::disable_if<boost::type_traits::
      ice_or<boost::is_base_of<ros::Duration, T>::value, boost::is_base_of<
      ros::Time, T>::value> >::type* = 0);
      
    /** \brief Retrieve the built-in variant's numeric value (overloaded
      *   version taking a non-numeric value)
      */
    template <typename T> static double getNumericValue(const T& value,
      typename boost::disable_if<typename type_traits::BuiltinType<T>::
      IsNumeric>::type* = 0);
    
    /** \brief Compare the values of two built-in variants (overloaded
      *   version taking two comparable values)
      */
    template <typename T> static bool isEqual(const typename type_traits::
      BuiltinType<T>::ValueType& lhs, const typename type_traits::
      BuiltinType<T>::ValueType& rhs, typename boost::enable_if<boost::
      has_equal_to<typename type_traits::BuiltinType<T>::ValueType,
      typename type_traits::BuiltinType<T>::ValueType, bool> >::type* = 0);
    
    /** \brief Compare the values of two built-in variants (overloaded
      *   version taking two non-comparable values)
      */
    template <typename T> static bool isEqual(const typename type_traits::
      BuiltinType<T>::ValueType& lhs, const typename type_traits::
      BuiltinType<T>::ValueType& rhs, typename boost::disable_if<boost::
      has_equal_to<typename type_traits::BuiltinType<T>::ValueType,
      typename type_traits::BuiltinType<T>::ValueType, bool> >::type* = 0);
    
    /** \brief Read a built-in variant from a stream (overloaded version
      *   taking a boolean value)
      */
    template <typename T> static void read(std::istream& stream,
      typename type_traits::BuiltinType<T>::ValueType& value, typename
      boost::enable_if<boost::is_same<T, bool> >::type* = 0);
    
    /** \brief Read a built-in variant from a stream (overloaded version
      *   taking a non-boolean, stream-readable value)
      */
    template <typename T> static void read(std::istream& stream,
      typename type_traits::BuiltinType<T>::ValueType& value, typename
      boost::disable_if<boost::is_same<T, bool> >::type* = 0, typename
      boost::enable_if<boost::has_right_shift<std::istream, typename
      type_traits::BuiltinType<T>::ValueType&> >::type* = 0);
    
    /** \brief Read a built-in variant from a stream (overloaded version
      *   taking a non-boolean, non-stream-readable value)
      */
    template <typename T> static void read(std::istream& stream,
      typename type_traits::BuiltinType<T>::ValueType& value, typename
      boost::disable_if<boost::is_same<T, bool> >::type* = 0, typename
      boost::disable_if<boost::has_right_shift<std::istream, typename
      type_traits::BuiltinType<T>::ValueType&> >::type* = 0);

    /** \brief Write a variant to a stream (overloaded version taking
      *   a boolean value)
      */
    template <typename T> static void write(std::ostream& stream, const
      typename type_traits::BuiltinType<T>::ValueType& value, typename
      boost::enable_if<boost::is_same<T, bool> >::type* = 0);
    
    /** \brief Write a variant to a stream (overloaded version taking
      *   a non-boolean, stream-writable value)
      */
    template <typename T> static void write(std::ostream& stream, const
      typename type_traits::BuiltinType<T>::ValueType& value, typename
      boost::disable_if<boost::is_same<T, bool> >::type* = 0, typename
      boost::enable_if<boost::has_left_shift<std::ostream, const typename
      type_traits::BuiltinType<T>::ValueType&> >::type* = 0);
    
    /** \brief Write a variant to a stream (overloaded version taking
      *   a non-boolean, non-stream-writable value)
      */
    template <typename T> static void write(std::ostream& stream, const
      typename type_traits::BuiltinType<T>::ValueType& value, typename
      boost::disable_if<boost::is_same<T, bool> >::type* = 0, typename
      boost::disable_if<boost::has_left_shift<std::ostream, const typename
      type_traits::BuiltinType<T>::ValueType&> >::type* = 0);
  };
};

#include <variant_topic_tools/BuiltinVariant.tpp>

#endif
