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

#include <boost/unordered_map.hpp>

#include <ros/ros.h>

namespace variant_topic_tools {
  /** \brief Data type information
    */
  class DataType {
  friend struct boost::hash<DataType>;
  public:
    /** \brief Default constructor
      */ 
    DataType();
    
    /** \brief Constructor for creating a data type by identifier
      * 
      * \note If the data type does not exist, it will be constructed as
      *   invalid data type.
      */ 
    DataType(const char* identifier);
    
    /** \brief Copy constructor
      */ 
    DataType(const DataType& src);
    
    /** \brief Destructor
      */ 
    ~DataType();

    /** \brief Retrieve the identifier representing this data type
      */ 
    std::string getIdentifier() const;
    
    /** \brief Retrieve the type information associated with this data type
      */ 
    const std::type_info& getInfo() const;
    
    /** \brief Retrieve the size of the instances of this data type
      * 
      * \note For variable-size data types, the reported size is zero.
      */ 
    size_t getSize() const;
    
    /** \brief Retrieve the member type of an array data type
      */ 
    DataType getMemberType() const;
    
    /** \brief Retrieve the member types of a complex data type in their
      *   defined order
      */ 
    std::vector<DataType> getMemberTypes() const;
    
    /** \brief True, if this data type represents a primitive data type,
      *   as opposed to a compound data type
      */ 
    bool isPrimitive() const;
    
    /** \brief True, if this data type represents a simple data type
      *  
      * A "simple" data type is one whose instances, given a vector/array
      * of them, can be memcpy'd directly.
      */ 
    bool isSimple() const;
    
    /** \brief True, if this data type represents a fixed-size data type
      *  
      * A fixed-size data type is one whose instances always have the same
      * size.
      */ 
    bool isFixedSize() const;
    
    /** \brief True, if this data type represents an arithmetic data type
      */ 
    bool isArithmetic() const;
    
    /** \brief True, if this data type represents a string data type
      */ 
    bool isString() const;
    
    /** \brief True, if this data type represents an array data type
      */ 
    bool isArray() const;
    
    /** \brief True, if this data type represents a message type
      */ 
    bool isMessage() const;
    
    /** \brief True, if this data type is valid
      */ 
    bool isValid() const;
    
    /** \brief Clear the data type
      */
    void clear();
    
    /** \brief Write the data type to a stream
      */
    void write(std::ostream& stream) const;
    
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const DataType& dataType) const {
      return (impl < dataType.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const DataType& dataType) const {
      return (impl == dataType.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const DataType& dataType) const {
      return (impl != dataType.impl);
    };
    
  protected:
    /** \brief Forward declaration of the data type implementation type
      */
    class Impl;
    
    /** \brief Declaration of the data type implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the data type implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief Data type implementation
      */
    class Impl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& identifier);
      
      /** \brief Destructor
        */
      virtual ~Impl();

      /** \brief Retrieve the type information associated with this data type
        *   (abstract declaration)
        */ 
      virtual const std::type_info& getInfo() const = 0;
    
      /** \brief Retrieve the size of the instances of this data type
        *   (abstract declaration)
        */
      virtual size_t getSize() const = 0;
      
      /** \brief True, if this data type represents a primitive data type,
        *   as opposed to a complex data type (abstract declaration)
        */
      virtual bool isPrimitive() const = 0;
      
      /** \brief True, if this data type represents a simple data type
        *   (abstract declaration)
        */
      virtual bool isSimple() const = 0;
      
      /** \brief True, if this data type represents a fixed-size data type
        *   (abstract declaration)
        */
      virtual bool isFixedSize() const = 0;
      
      /** \brief True, if this data type represents an arithmetic data type
        *   (abstract declaration)
        */ 
      virtual bool isArithmetic() const = 0;
    
      /** \brief True, if this data type represents a string data type
        *   (abstract declaration)
        */ 
      virtual bool isString() const = 0;
      
      /** \brief True, if this data type represents an array data type
        *   (abstract declaration)
        */ 
      virtual bool isArray() const = 0;
    
      /** \brief True, if this data type represents an array data type
        *   (abstract declaration)
        */ 
      virtual bool isMessage() const = 0;
    
      /** \brief True, if this data type is valid
        */
      bool isValid() const;

      /** \brief The identifier representing this data type
        */
      std::string identifier;
      
      /** \brief The member types of this data type
        */
      std::vector<DataType> memberTypes;        
    };
    
    /** \brief Data type implementation (templated version)
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
        *   (implementation)
        */ 
      const std::type_info& getInfo() const;
    
      /** \brief Retrieve the size of the instances of this data type
        *   (implementation)
        */
      size_t getSize() const;
      
      /** \brief True, if this data type represents a primitive data type,
        *   as opposed to a complex data type (implementation)
        */
      bool isPrimitive() const;
      
      /** \brief True, if this data type represents a simple data type
        *   (implementation)
        */
      bool isSimple() const;
      
      /** \brief True, if this data type represents a fixed-size data type
        *   (implementation)
        */
      bool isFixedSize() const;
      
      /** \brief True, if this data type represents an arithmetic data type
        *   (implementation)
        */ 
      bool isArithmetic() const;
      
      /** \brief True, if this data type represents a string data type
        *   (implementation)
        */ 
      bool isString() const;
      
      /** \brief True, if this data type represents an array data type
        *   (implementation)
        */ 
      bool isArray() const;
      
      /** \brief True, if this data type represents a message type
        *   (implementation)
        */ 
      bool isMessage() const;
    };
    
    /** \brief Data type instances
      */
    class Instances:
      public boost::unordered_map<std::string, DataType> {
    public:
      /** \brief Default constructor
        */
      Instances();
      
      /** \brief Destructor
        */
      ~Instances();
      
      /** \brief Add a non-message type
        */
      template <typename T> void add(const std::string& identifier);
      
      /** \brief Add a message type
        */
      template <typename T> void add();
    };
    
    /** \brief Access the data type instances
      */ 
    static Instances& getInstances();
    
    /** \brief The data type's implementation
      */
    ImplPtr impl;
  };
  
  std::ostream& operator<<(std::ostream& stream, const DataType&
    dataType);
};

namespace boost {
  /** \brief Specialization of the data type hash function
    */
  template <> struct hash<variant_topic_tools::DataType> {
    size_t operator()(const variant_topic_tools::DataType& dataType)
        const {
      return reinterpret_cast<size_t>(dataType.impl.get());
    };
  };
}

#include <variant_topic_tools/DataType.tpp>

#endif
