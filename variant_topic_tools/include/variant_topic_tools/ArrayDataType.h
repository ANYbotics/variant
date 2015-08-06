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

#include <vector>

#include <boost/array.hpp>

#include <variant_topic_tools/DataType.h>

namespace variant_topic_tools {
  /** \brief Array data type
    */
  class ArrayDataType :
    public DataType {
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
    ~ArrayDataType();
  
    /** \brief Retrieve the element type of this array data type
      */
    const DataType& getElementType() const;
    
    /** \brief Retrieve the number of elements of this array data type
      */
    size_t getNumElements() const;
    
  protected:
    /** \brief Type traits
      */
    struct TypeTraits {
      template <class A> struct FromArray;
      
      template <typename T> struct FromArray<std::vector<T> > {
        typedef T ElementType;
        static const size_t NumElements = 0;
      };
      
      template <typename T, size_t N> struct FromArray<boost::array<T, N> > {
        typedef T ElementType;
        static const size_t NumElements = N;
      };
      
      template <typename T, size_t N> struct ToArray {
        typedef boost::array<T, N> ArrayType;
      };
      
      template <typename T> struct ToArray<T, 0> {
        typedef std::vector<T> ArrayType;
      };
    };
    
    /** \brief Array data type implementation
      */
    class Impl :
      public virtual DataType::Impl {
    public:
      /** \brief Constructor
        */
      Impl(const DataType& elementType);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the identifier representing this data type
        *   (implementation)
        */ 
      const std::string& getIdentifier() const;
    
      /** \brief Retrieve the number of elements of this array data type
        *   (abstract declaration)
        */
      virtual size_t getNumElements() const = 0;
      
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
      
      /** \brief The element type of the array data type
        */
      DataType elementType;
    };
    
    /** \brief Array data type implementation (variant-typed version)
      */
    class ImplV :
      public DataType::ImplV,
      public Impl {
    public:
      /** \brief Constructor
        */
      ImplV(const DataType& elementType, size_t numElements = 0);
      
      /** \brief Destructor
        */
      virtual ~ImplV();
      
      /** \brief Retrieve the number of elements of this array data type
        *   (implementation)
        */
      size_t getNumElements() const;
      
      /** \brief The number of elements of the array data type
        */
      size_t numElements;
    };
    
    /** \brief Array data type implementation (templated strong-typed version)
      */
    template <typename T, size_t N> class ImplT :
      public DataType::ImplT<typename TypeTraits::ToArray<T, N>::ArrayType>,
      public Impl {
    public:
      /** \brief Constructor
        */
      ImplT();
      
      /** \brief Destructor
        */
      virtual ~ImplT();
      
      /** \brief Retrieve the number of elements of this array data type
        *   (implementation)
        */
      size_t getNumElements() const;
    };
    
    /** \brief Constructor (overloaded version taking an element type
      *   and a number of elements)
      */ 
    ArrayDataType(const DataType& elementType, size_t numElements);
    
    /** \brief Create an array data type (version templated on the
      *   array type)
      */ 
    template <class A> static ArrayDataType create();
    
    /** \brief Create an array data type (version templated on the
      *   element type and the number of elements)
      */ 
    template <typename T, size_t N> static ArrayDataType create();
  };
};

#include <variant_topic_tools/ArrayDataType.tpp>

#endif
