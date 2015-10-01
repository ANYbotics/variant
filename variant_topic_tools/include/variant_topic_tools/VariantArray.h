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

/** \file VariantArray.h
  * \brief Header file providing the VariantArray class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_VARIANT_ARRAY_H
#define VARIANT_TOPIC_TOOLS_VARIANT_ARRAY_H

#include <vector>

#include <variant_topic_tools/VariantCollection.h>

namespace variant_topic_tools {
  /** \brief Variant array type
    */  
  class VariantArray :
    public VariantCollection {
  friend class ArrayDataType;
  friend class Variant;
  public:
    /** \brief Default constructor
      */ 
    VariantArray();
    
    /** \brief Copy constructor
      */ 
    VariantArray(const VariantArray& src);
    
    /** \brief Copy constructor (overloaded version taking a variant)
      */ 
    VariantArray(const Variant& src);
    
    /** \brief Destructor
      */ 
    ~VariantArray();
        
    /** \brief True, if this variant array is a fixed size array
      */ 
    bool isFixedSize() const;
    
    /** \brief Add a member to the variant array
      * 
      * \note An attempt to adding a member to a fixed-size array will result
      *   in an exception being thrown. Further, the data type of the variant
      *   member to be added must match the member type of this array.
      */ 
    void addMember(const Variant& member);
  
    /** \brief Resize the variant array
      * 
      * \note An attempt to resizing a fixed-size array will result in an
      *   exception being thrown.
      */ 
    void resize(size_t numMembers);
    
    /** \brief Clear the variant array
      * 
      * \note An attempt to clearing a fixed-size array will result in an
      *   exception being thrown.
      */ 
    void clear();
    
    /** \brief Operator for adding a member to the variant array
      */ 
    VariantArray& operator+=(const Variant& member);
    
  protected:
    /** \brief Type traits
      */
    struct TypeTraits {
      template <typename T, size_t N> struct ToArray {
        typedef boost::array<T, N> ArrayType;
        
        static void add(ArrayType& array, const T& element);
        static void resize(ArrayType& array, size_t numElements);
        static void clear(ArrayType& array);
      };
      
      template <typename T> struct ToArray<T, 0> {
        typedef std::vector<T> ArrayType;
        
        static void add(ArrayType& array, const T& element);
        static void resize(ArrayType& array, size_t numElements);
        static void clear(ArrayType& array);
      };
    };
    
    /** \brief Variant array value (abstract base)
      */
    class Value :
      public VariantCollection::Value {
    public:
      /** \brief Default constructor
        */ 
      Value();
      
      /** \brief Destructor
        */ 
      virtual ~Value();
      
      /** \brief Set a member of the variant collection by name
        *   (implementation)
        */
      void setMember(const std::string& name, const Variant& member);
      
      using VariantCollection::Value::setMember;
      
      /** \brief Retrieve a member of the variant collection by name
        *   (implementation)
        */
      SharedVariant getMember(const std::string& name) const;
      
      using VariantCollection::Value::getMember;
      
      /** \brief True, if the variant array is a fixed size array (abstract
        *   declaration)
        */ 
      virtual bool isFixedSize() const = 0;
      
      /** \brief True, if the variant collection contains the member with the
        *   specified name (implementation)
        */
      bool hasMember(const std::string& name) const;
    
      /** \brief Write the variant collection member with the specified
        *   index to a stream (implementation)
        */
      void writeMember(std::ostream& stream, size_t index) const;
      
      /** \brief Add a member to the variant array (abstract declaration)
        */ 
      virtual void addMember(const Variant& member) = 0;
  
      /** \brief Resize the variant array (abstract declaration)
        */ 
      virtual void resize(size_t numMembers) = 0;
      
      /** \brief Clear the variant array (abstract declaration)
        */ 
      virtual void clear() = 0;
    };
    
    /** \brief Variant array value (variant-typed implementation)
      */
    class ValueImplV :
      public Value {
    public:
      /** \brief Default constructor
        */ 
      ValueImplV(const DataType& memberType = DataType(), size_t
        numMembers = 0);
      
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
      void setMember(size_t index, const Variant& member);
      
      /** \brief Retrieve a member of the variant collection by index
        *   (implementation)
        */
      SharedVariant getMember(size_t index) const;
      
      /** \brief True, if the variant array is a fixed size array
        *   (implementation)
        */ 
      bool isFixedSize() const;
      
      /** \brief Add a member to the variant array (implementation)
        */ 
      void addMember(const Variant& member);
  
      /** \brief Resize the variant array (implementation)
        */ 
      void resize(size_t numMembers);
      
      /** \brief Clear the variant array (implementation)
        */ 
      void clear();
      
      /** \brief Clone this variant value (implementation)
        */
      ValuePtr clone() const;
      
      /** \brief The array member type
        */
      DataType memberType;
      
      /** \brief The number of array members
        */
      size_t numMembers;
      
      /** \brief The array members
        */
      std::vector<Variant> members;
    };
    
    /** \brief Variant array value (templated implementation)
      */
    template <typename T, size_t N> class ValueImplT :
      public Variant::ValueT<typename VariantArray::TypeTraits::
        ToArray<T, N>::ArrayType>,
      public Value {
    public:
      /** \brief Declaration of the array type
        */ 
      typedef typename VariantArray::TypeTraits::ToArray<T, N>::ArrayType
        ArrayType;
      
      /** \brief Declaration of the array pointer type
        */ 
      typedef boost::shared_ptr<ArrayType> ArrayPtr;
      
      /** \brief Declaration of the array weak pointer type
        */ 
      typedef boost::weak_ptr<ArrayType> ArrayWPtr;
        
      /** \brief Default constructor
        */ 
      ValueImplT(const DataType& memberType = DataType(), const ArrayType&
        members = ArrayType());
      
      /** \brief Copy constructor
        */ 
      ValueImplT(const ValueImplT<T, N>& src);
      
      /** \brief Destructor
        */ 
      virtual ~ValueImplT();
      
      /** \brief Set the variant's value (implementation)
        */
      void setValue(const ArrayType& value);
      
      /** \brief Retrieve the variant's value (implementation of the
        *   non-const version)
        */
      ArrayType& getValue();
      
      /** \brief Retrieve the variant's value (implementation of the
        *   const version)
        */
      const ArrayType& getValue() const;
      
      /** \brief Retrieve the number of members of the variant collection
        *   (implementation)
        */
      size_t getNumMembers() const;
      
      /** \brief Set a member of the variant collection by index
        *   (implementation)
        */
      void setMember(size_t index, const Variant& member);
      
      /** \brief Retrieve a member of the variant collection by index
        *   (implementation)
        */
      SharedVariant getMember(size_t index) const;
      
      /** \brief True, if the variant array is a fixed size array
        *   (implementation)
        */ 
      bool isFixedSize() const;
      
      /** \brief True, if this variant value equals another variant value
        *   (re-implementation)
        */
      bool isEqual(const Variant::Value& value) const;
      
      /** \brief Add a member to the variant array (implementation)
        */ 
      void addMember(const Variant& member);
  
      /** \brief Resize the variant array (implementation)
        */ 
      void resize(size_t numMembers);
      
      /** \brief Clear the variant array (implementation)
        */ 
      void clear();
      
      /** \brief Clone this variant value (implementation)
        */
      ValuePtr clone() const;
      
      /** \brief Read the variant from a stream (re-implementation)
        */
      void read(std::istream& stream);
    
      /** \brief Write this variant value to a stream (re-implementation)
        */
      void write(std::ostream& stream) const;
      
      /** \brief The array member type
        */
      DataType memberType;
      
      /** \brief The array members
        */
      ArrayPtr members;
    };
    
    /** \brief Constructor (overloaded version taking an array data type
      *   and a size)
      */
    VariantArray(const ArrayDataType& type, size_t numMembers);
    
    /** \brief Create a variant array
      */ 
    template <typename T, size_t N> static VariantArray create();
  };
};

#include <variant_topic_tools/VariantArray.tpp>

#endif
