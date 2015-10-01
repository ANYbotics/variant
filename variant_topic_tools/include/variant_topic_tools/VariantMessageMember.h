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

/** \file VariantMessageMember.h
  * \brief Header file providing the VariantMessageMember class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_VARIANT_MESSAGE_MEMBER_H
#define VARIANT_TOPIC_TOOLS_VARIANT_MESSAGE_MEMBER_H

#include <variant_topic_tools/Variant.h>

namespace variant_topic_tools {
  /** \brief Variant message member type
    */  
  class VariantMessageMember :
    public Variant {
  friend class VariantMessage;
  public:
    /** \brief Default constructor
      */ 
    VariantMessageMember();
    
    /** \brief Copy constructor
      */ 
    VariantMessageMember(const VariantMessageMember& src);
    
    /** \brief Copy constructor (overloaded version taking a variant)
      */ 
    VariantMessageMember(const Variant& src);
    
    /** \brief Destructor
      */ 
    ~VariantMessageMember();        
    
  protected:
    /** \brief Variant message member value
      */
    template <typename T> class ValueImplT :
      public Variant::ValueT<T> {
    public:
      /** \brief Declaration of the array type
        */ 
      typedef typename VariantMessageMember::TypeTraits::ToArray<T, N>::
        ArrayType ArrayType;
      
      /** \brief Declaration of the array pointer type
        */ 
      typedef boost::shared_ptr<ArrayType> ArrayPtr;
      
      /** \brief Declaration of the array weak pointer type
        */ 
      typedef boost::weak_ptr<ArrayType> ArrayWPtr;
      
      /** \brief Default constructor
        */ 
      ValueImplT(const ArrayPtr& array, size_t index);
      
      /** \brief Copy constructor
        */ 
      ValueImplT(const ValueImplT<T, N>& src);
      
      /** \brief Destructor
        */ 
      virtual ~ValueImplT();
      
      /** \brief Set the variant's value (implementation)
        */
      void setValue(const T& value);
      
      /** \brief Retrieve the variant's value (implementation of the
        *   non-const version)
        */
      T& getValue();
      
      /** \brief Retrieve the variant's value (implementation of the
        *   const version)
        */
      const T& getValue() const;
      
      /** \brief Clone this variant value (implementation)
        */
      ValuePtr clone() const;
      
      /** \brief The variant array holding this array member
        */
      ArrayWPtr array;
      
      /** \brief The index of the array member value
        */
      size_t index;
    };
    
    /** \brief Create a variant array member
      */ 
    template <typename T, size_t N> static VariantMessageMember create(const
      DataType& type, const typename VariantMessageMember::ValueImplT<T, N>::
      ArrayPtr& array, size_t index);
  };
};

#include <variant_topic_tools/VariantMessageMember.tpp>

#endif
