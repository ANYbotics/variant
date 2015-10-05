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
    };
    
    /** \brief Built-in variant value (templated implementation)
      */
    template <typename T> class ValueT :
      public Variant::ValueT<T>,
      public Value {
    public:
      /** \brief Default constructor
        */ 
      ValueT(const T& value = T());
      
      /** \brief Copy constructor
        */ 
      ValueT(const ValueT<T>& src);
      
      /** \brief Destructor
        */ 
      virtual ~ValueT();
      
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
      
      /** \brief The strong-typed value
        */
      T value;
    };
    
    /** \brief Create a built-in variant
      */ 
    template <typename T> static BuiltinVariant create(const DataType& type);
  };
};

#include <variant_topic_tools/BuiltinVariant.tpp>

#endif
