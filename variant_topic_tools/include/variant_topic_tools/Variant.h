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

/** \file Variant.h
  * \brief Header file providing the Variant class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_VARIANT_H
#define VARIANT_TOPIC_TOOLS_VARIANT_H

#include <boost/type_traits.hpp>

#include <ros/ros.h>

#include <variant_topic_tools/DataType.h>

namespace variant_topic_tools {
  /** \brief Variant type
    */  
  class Variant {
  public:
    /** \brief Default constructor
      */ 
    Variant();
    
    /** \brief Copy constructor
      */ 
    template <typename T> Variant(const T& src);
    
    /** \brief Destructor
      */ 
    ~Variant();
    
    /** \brief Set the variant's value
      */
    template <typename T> void setValue(const T& value);
    
    /** \brief Retrieve the variant's value (non-const version)
      */
    template <typename T> T& getValue();
    
    /** \brief Retrieve the variant's value (const version)
      */
    template <typename T> const T& getValue() const;
    
    /** \brief Access the type of the variant's value
      */
    const DataType& getType() const;
    
    /** \brief True, if the variant has a type
      */
    bool hasType() const;
    
    /** \brief True, if this variant represents an array
      */ 
    bool isArray() const;
    
    /** \brief True, if this variant represents a built-in
      */ 
    bool isBuiltin() const;
    
    /** \brief True, if this variant represents a collection
      */ 
    bool isCollection() const;
    
    /** \brief True, if this variant represents a message
      */ 
    bool isMessage() const;
    
    /** \brief True, if the variant is empty, i.e., does not have a value
      *   assigned
      */
    bool isEmpty() const;

    /** \brief Retrieve this variant as array
      */ 
    ArrayVariant asArray() const;
    
    /** \brief Retrieve this variant as builtin
      */ 
    BuiltinVariant asBuiltin() const;
    
    /** \brief Retrieve this variant as collection
      */ 
    CollectionVariant asCollection() const;
    
    /** \brief Retrieve this variant as message
      */ 
    MessageVariant asMessage() const;
    
    /** \brief Clear the variant
      */
    void clear();
    
    /** \brief Read the variant from a stream
      */
    void read(std::istream& stream);
    
    /** \brief Write the variant to a stream
      */
    void write(std::ostream& stream) const;
    
    /** \brief Assignment operator
      */
    template <typename T> Variant& operator=(const T& src);
    
    /** \brief Conversion operator
      */
    template <typename T> operator const T&() const;
    
    /** \brief True, if this variant is equal to another variant
      */
    bool operator==(const Variant& variant) const;
    
    /** \brief True, if this variant is not equal to another variant
      */
    bool operator!=(const Variant& variant) const;
    
  protected:
    /** \brief Type traits
      */
    struct TypeTraits {
      template <typename T> static void assign(Variant& dst, const T& src,
        typename boost::enable_if<boost::is_base_of<Variant, T> >::type*
        dummy = 0);
      template <typename T> static void assign(Variant& dst, const T& src,
        typename boost::disable_if<boost::is_base_of<Variant, T> >::type*
        dummy = 0);
      
      template <typename T> static bool isEqual(const T& lhs, const T& rhs,
        typename boost::enable_if<boost::has_equal_to<T, T, bool> >::type*
        dummy = 0);
      template <typename T, size_t N> static bool isEqual(const
        boost::array<T, N>& lhs, const boost::array<T, N>& rhs,
        typename boost::enable_if<boost::has_equal_to<T, T, bool> >::type*
        dummy = 0);
      template <typename T> static bool isEqual(const std::vector<T>& lhs,
        const std::vector<T>& rhs, typename boost::enable_if<boost::
        has_equal_to<T, T, bool> >::type* dummy = 0);
      template <typename T> static bool isEqual(const T& lhs, const T& rhs,
        typename boost::disable_if<boost::has_equal_to<T, T, bool> >::type*
        dummy = 0);
      template <typename T, size_t N> static bool isEqual(const
        boost::array<T, N>& lhs, const boost::array<T, N>& rhs,
        typename boost::disable_if<boost::has_equal_to<T, T, bool> >::type*
        dummy = 0);
      template <typename T> static bool isEqual(const std::vector<T>& lhs,
        const std::vector<T>& rhs, typename boost::disable_if<boost::
        has_equal_to<T, T, bool> >::type* dummy = 0);
        
      template <typename T> static void read(std::istream& stream, T& value,
        typename boost::enable_if<boost::has_right_shift<std::istream, T&> >::
        type* dummy = 0);
      template <typename T> static void read(std::istream& stream, T& value,
        typename boost::disable_if<boost::has_right_shift<std::istream, T&> >::
        type* dummy = 0);
      
      template <typename T> static void write(std::ostream& stream, const T&
        value, typename boost::enable_if<boost::has_left_shift<std::ostream,
        const T&> >::type* dummy = 0);
      template <typename T> static void write(std::ostream& stream, const T&
        value, typename boost::disable_if<boost::has_left_shift<std::ostream,
        const T&> >::type* dummy = 0);
    };
    
    /** \brief Forward declaration of the variant value type
      */
    class Value;
    
    /** \brief Declaration of the variant value pointer type
      */
    typedef boost::shared_ptr<Value> ValuePtr;
    
    /** \brief Declaration of the variant value weak pointer type
      */
    typedef boost::weak_ptr<Value> ValueWPtr;
    
    /** \brief Variant value (abstract base)
      */
    class Value {
    public:
      /** \brief Default constructor
        */ 
      Value();
      
      /** \brief Destructor
        */ 
      virtual ~Value();
      
      /** \brief True, if this variant value equals another variant value
        *   (abstract declaration)
        */
      virtual bool isEqual(const Value& value) const = 0;
      
      /** \brief Clone this variant value (abstract declaration)
        */
      virtual ValuePtr clone() const = 0;    
      
      /** \brief Read the variant from a stream (abstract declaration)
        */
      virtual void read(std::istream& stream) = 0;
    
      /** \brief Write this variant value to a stream (abstract declaration)
        */
      virtual void write(std::ostream& stream) const = 0;    
    };

    /** \brief Variant value (templated abstract base)
      */
    template <typename T> class ValueT :
      public virtual Value {
    public:
      /** \brief Default constructor
        */ 
      ValueT();
      
      /** \brief Destructor
        */ 
      virtual ~ValueT();
      
      /** \brief Set the variant's value (abstract declaration)
        */
      virtual void setValue(const T& value) = 0;
      
      /** \brief Retrieve the variant's value (abstract declaration of the
        *   non-const version)
        */
      virtual T& getValue() = 0;
      
      /** \brief Retrieve the variant's value (abstract declaration of the
        *   const version)
        */
      virtual const T& getValue() const = 0;
      
      /** \brief True, if this variant value equals another variant value
        *   (implementation)
        */
      bool isEqual(const Value& value) const;
      
      /** \brief Read the variant from a stream (implementation)
        */
      void read(std::istream& stream);
    
      /** \brief Write this variant value to a stream (implementation)
        */
      void write(std::ostream& stream) const;
    };

    /** \brief The variant's data type
      */
    DataType type;
    
    /** \brief The variant's data value
      */
    ValuePtr value;
    
    /** \brief Constructor (overloaded version taking a data type)
      */ 
    Variant(const DataType& type);
  };
  
  /** \brief Operator for reading the variant from a stream
    */
  std::istream& operator>>(std::istream& stream, Variant& variant);
  
  /** \brief Operator for writing the variant to a stream
    */
  std::ostream& operator<<(std::ostream& stream, const Variant& variant);
};

#include <variant_topic_tools/Variant.tpp>

#endif
