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

/** \file ArrayDataType.h
  * \brief Header file providing the ArrayDataType class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_ARRAY_DATA_TYPE_H
#define VARIANT_TOPIC_TOOLS_ARRAY_DATA_TYPE_H

#include <variant_topic_tools/ArrayTypeTraits.h>
#include <variant_topic_tools/DataType.h>

namespace variant_topic_tools {
  /** \brief Array data type
    */
  class ArrayDataType :
    public DataType {
  friend class ArraySerializer;
  friend class ArrayVariant;
  friend class DataType;
  friend class DataTypeRegistry;
  public:
    /** \brief Default constructor
      */ 
    ArrayDataType();
    
    /** \brief Copy constructor
      */ 
    ArrayDataType(const ArrayDataType& src);
    
    /** \brief Copy constructor (overloaded version taking a data type)
      */ 
    ArrayDataType(const DataType& src);
    
    /** \brief Destructor
      */ 
    virtual ~ArrayDataType();
  
    /** \brief Retrieve the member type of this array data type
      */
    const DataType& getMemberType() const;
    
    /** \brief Retrieve the number of members of this array data type
      */
    size_t getNumMembers() const;
    
    /** \brief Assignment operator
      */
    ArrayDataType& operator=(const DataType& src);
    
  protected:
    /** \brief Array data type implementation
      */
    class Impl :
      public DataType::Impl {
    public:
      /** \brief Constructor
        */
      Impl(const DataType& memberType);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the identifier representing this data type
        *   (implementation)
        */ 
      const std::string& getIdentifier() const;
    
      /** \brief Retrieve the number of members of this array data type
        *   (abstract declaration)
        */
      virtual size_t getNumMembers() const = 0;
      
      /** \brief Retrieve the size of the instances of this data type
        *   (implementation)
        */
      size_t getSize() const;
      
      /** \brief True, if this data type represents a fixed-size data type,
        *   as opposed to a variable-size data type (implementation)
        */ 
      bool isFixedSize() const;
      
      /** \brief The identifier representing this array data type
        */
      mutable std::string identifier;
      
      /** \brief The member type of the array data type
        */
      DataType memberType;
    };
    
    /** \brief Array data type implementation (variant-typed version)
      */
    class ImplV :
      public Impl {
    public:
      /** \brief Constructor
        */
      ImplV(const DataType& memberType, size_t numMembers);
      
      /** \brief Destructor
        */
      virtual ~ImplV();
      
      /** \brief Retrieve the number of members of this array data type
        *   (implementation)
        */
      size_t getNumMembers() const;
      
      /** \brief Create a serializer for this data type (re-implementation)
        */ 
      Serializer createSerializer(const DataType& type) const;
      
      /** \brief Create a variant from this data type (re-implementation)
        */ 
      Variant createVariant(const DataType& type) const;
      
      /** \brief The number of members of the array data type
        */
      size_t numMembers;
    };
    
    /** \brief Array data type implementation (templated strong-typed version)
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
      
      /** \brief Retrieve the type information associated with this data type
        *   (re-implementation)
        */ 
      const std::type_info& getTypeInfo() const;
      
      /** \brief Retrieve the number of members of this array data type
        *   (implementation)
        */
      size_t getNumMembers() const;
      
      /** \brief Create a serializer for this data type (re-implementation)
        */ 
      Serializer createSerializer(const DataType& type) const;
      
      /** \brief Create a variant from this data type (re-implementation)
        */ 
      Variant createVariant(const DataType& type) const;
    };
    
    /** \brief Constructor (overloaded version taking an member type
      *   and a number of members)
      */ 
    ArrayDataType(const DataType& memberType, size_t numMembers);
    
    /** \brief Create an array data type (version templated on the
      *   array type)
      */ 
    template <typename T> static ArrayDataType create();
    
    /** \brief Create an array data type (overloaded version for creating
      *   a non-fixed-size array data type)
      */ 
    template <typename T, size_t N> static ArrayDataType create(typename
      boost::enable_if<boost::type_traits::ice_eq<N, 0> >::type* = 0);
    
    /** \brief Create an array data type (overloaded version for creating
      *   a fixed-size array data type)
      */ 
    template <typename T, size_t N> static ArrayDataType create(typename
      boost::disable_if<boost::type_traits::ice_eq<N, 0> >::type* = 0);
  };
};

#include <variant_topic_tools/ArrayDataType.tpp>

#endif
