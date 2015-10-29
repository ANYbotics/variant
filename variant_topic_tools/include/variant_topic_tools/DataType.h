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

/** \file DataType.h
  * \brief Header file providing the DataType class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_DATA_TYPE_H
#define VARIANT_TOPIC_TOOLS_DATA_TYPE_H

#include <typeinfo>
#include <vector>

#include <ros/ros.h>

#include <variant_topic_tools/Forwards.h>

namespace variant_topic_tools {
  /** \brief Data type
    */
  class DataType {
  friend class DataTypeHash;
  friend class DataTypeRegistry;
  friend class Serializer;
  public:
    /** \brief Default constructor
      */ 
    DataType();
    
    /** \brief Constructor (overloaded version taking an identifier)
      * 
      * \note If the data type matching the specified identifier cannot
      *   be found in the data type registry, the instantiated data
      *   type will be invalid.
      */ 
    DataType(const char* identifier);
    
    /** \brief Constructor (overloaded version taking a string identifier)
      * 
      * \note If the data type matching the specified identifier cannot
      *   be found in the data type registry, the instantiated data
      *   type will be invalid.
      */ 
    DataType(const std::string& identifier);
    
    /** \brief Constructor (overloaded version taking type information)
      * 
      * \note If the data type matching the specified type information
      *   cannot be found in the data type registry, the instantiated
      *   data type will be invalid.
      */ 
    DataType(const std::type_info& typeInfo);
    
    /** \brief Copy constructor
      */ 
    DataType(const DataType& src);
    
    /** \brief Destructor
      */ 
    virtual ~DataType();

    /** \brief Retrieve the identifier representing this data type
      */ 
    const std::string& getIdentifier() const;
    
    /** \brief Retrieve the type information associated with this data type
      *
      * \note If no type information is available for this data type,
      *   the type information for void will be returned. 
      */ 
    const std::type_info& getTypeInfo() const;
    
    /** \brief Retrieve the size of the instances of this data type
      * 
      * \note For variable-size data types, the reported size will be zero.
      */ 
    size_t getSize() const;
    
    /** \brief True, if this data type represents an array type
      */ 
    bool isArray() const;
    
    /** \brief True, if this data type represents a built-in type
      */ 
    bool isBuiltin() const;
    
    /** \brief True, if this data type represents a message type
      */ 
    bool isMessage() const;
    
    /** \brief True, if this data type represents a fixed-size data type,
      *   as opposed to a variable-size data type
      */ 
    bool isFixedSize() const;

    /** \brief True, if this data type represents a simple data type, i.e.,
      *   a data type such that an array of its instances can safely be
      *   shallow copied.
      */ 
    bool isSimple() const;

    /** \brief True, if this data type is valid
      */ 
    bool isValid() const;
    
    /** \brief True, if this data type has type information
      */ 
    bool hasTypeInfo() const;
    
    /** \brief Clear the data type
      */
    void clear();
    
    /** \brief Write the data type to a stream
      */
    void write(std::ostream& stream) const;
    
    /** \brief Create a serializer for this data type
      */ 
    Serializer createSerializer() const;
    
    /** \brief Create a variant from this data type
      */ 
    Variant createVariant() const;
    
    /** \brief Assignment operator
      */
    virtual DataType& operator=(const DataType& src);
    
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl) ? (void*)1 : (void*)0;
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const DataType& dataType) const {
      return (impl && dataType.impl) ? (*impl == *dataType.impl) :
        (impl == dataType.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const DataType& dataType) const {
      return (impl && dataType.impl) ? (*impl != *dataType.impl) :
        (impl != dataType.impl);
    };
    
  protected:
    /** \brief Data type implementation
      */
    class Impl {
    public:
      /** \brief Constructor
        */
      Impl();
      
      /** \brief Destructor
        */
      virtual ~Impl();

      /** \brief Retrieve the identifier representing this data type
        *   (abstract declaration)
        */ 
      virtual const std::string& getIdentifier() const = 0;
    
      /** \brief Retrieve the type information associated with this data type
        */ 
      virtual const std::type_info& getTypeInfo() const;
    
      /** \brief Retrieve the size of the instances of this data type
        *   (abstract declaration)
        */
      virtual size_t getSize() const = 0;
      
      /** \brief True, if this data type represents a fixed-size data type
        *   (abstract declaration)
        */ 
      virtual bool isFixedSize() const = 0;
      
      /** \brief True, if this data type represents a simple data type
        *   (abstract declaration)
        */ 
      virtual bool isSimple() const = 0;
      
      /** \brief Create a serializer for this data type (abstract declaration)
        */ 
      virtual Serializer createSerializer(const DataType& type) const = 0;
      
      /** \brief Create a variant from this data type (abstract declaration)
        */ 
      virtual Variant createVariant(const DataType& type) const = 0;
    };
    
    /** \brief Declaration of the data type implementation pointer type
      *
      * \note This pointer uses two layers of indirection to allow for
      *   updating the underlying implementation for all instances of
      *   data type pointing to the same implementation.
      */
    typedef boost::shared_ptr<boost::shared_ptr<Impl> > ImplPtr;
    
    /** \brief Declaration of the data type implementation weak pointer
      *   type
      */
    typedef boost::shared_ptr<boost::weak_ptr<Impl> > ImplWPtr;
    
    /** \brief The data type's implementation adapter
      */
    ImplPtr impl;
  };
    
  /** \brief Operator for writing the data type to a stream
    */
  std::ostream& operator<<(std::ostream& stream, const DataType& dataType);
};

#endif
