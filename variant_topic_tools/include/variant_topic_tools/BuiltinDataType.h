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

/** \file BuiltinDataType.h
  * \brief Header file providing the BuiltinDataType class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_BUILTIN_DATA_TYPE_H
#define VARIANT_TOPIC_TOOLS_BUILTIN_DATA_TYPE_H

#include <variant_topic_tools/DataType.h>

namespace variant_topic_tools {
  /** \brief Built-in data type
    */
  class BuiltinDataType :
    public DataType {
  friend class DataType;
  friend class DataTypeRegistry;
  public:
    /** \brief Default constructor
      */ 
    BuiltinDataType();
    
    /** \brief Copy constructor
      */ 
    BuiltinDataType(const BuiltinDataType& src);
    
    /** \brief Copy constructor (overloaded version taking a data type)
      */ 
    BuiltinDataType(const DataType& src);
    
    /** \brief Destructor
      */ 
    virtual ~BuiltinDataType();
    
    /** \brief Assignment operator
      */
    BuiltinDataType& operator=(const DataType& src);
    
  protected:
    /** \brief Built-in data type implementation
      */
    class Impl :
      public DataType::Impl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& identifier);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the identifier representing this data type
        *   (implementation)
        */ 
      const std::string& getIdentifier() const;
    
      /** \brief The identifier representing this built-in data type
        */
      std::string identifier;
    };
    
    /** \brief Built-in data type implementation (templated strong-typed
      *   version)
      */
    template <typename T> class ImplT :
      public Impl {
    public:
      /** \brief Constructor
        */
      ImplT(const std::string& identifier);
      
      /** \brief Destructor
        */
      virtual ~ImplT();
    
      /** \brief Retrieve the type information associated with this data type
        *   (re-implementation)
        */ 
      const std::type_info& getTypeInfo() const;
      
      /** \brief Retrieve the size of the instances of this data type
        *   (implementation)
        */
      size_t getSize() const;
      
      /** \brief True, if this data type represents a fixed-size data type,
        *   as opposed to a variable-size data type (implementation)
        */ 
      bool isFixedSize() const;
      
      /** \brief Create a serializer for this data type (re-implementation)
        */ 
      Serializer createSerializer() const;
      
      /** \brief Create a variant from this data type (re-implementation)
        */ 
      Variant createVariant() const;
    };
    
    /** \brief Create a built-in data type
      */ 
    template <typename T> static BuiltinDataType create(const
      std::string& identifier);    
  };
};

#include <variant_topic_tools/BuiltinDataType.tpp>

#endif
