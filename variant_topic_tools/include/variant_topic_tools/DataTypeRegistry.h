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

/** \file DataTypeRegistry.h
  * \brief Header file providing the DataTypeRegistry class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_DATA_TYPE_REGISTRY_H
#define VARIANT_TOPIC_TOOLS_DATA_TYPE_REGISTRY_H

#include <typeinfo>

#include <boost/unordered_map.hpp>

#include <variant_topic_tools/ArrayDataType.h>
#include <variant_topic_tools/BuiltinDataType.h>
#include <variant_topic_tools/MessageDataType.h>
#include <variant_topic_tools/TypeInfoHash.h>

namespace variant_topic_tools {
  /** \brief Data type registry
    */
  class DataTypeRegistry {
  friend class DataType;
  public:
    /** \brief Default constructor
      */ 
    DataTypeRegistry();
    
    /** \brief Destructor
      */ 
    ~DataTypeRegistry();

    /** \brief Retrieve a data type from the registry by identifier
      * 
      * \note If no data type matching the given identifier exists in the
      *   data type registry, the returned data type will be invalid.
      */
    DataType getDataType(const std::string& identifier) const;
    
    /** \brief Retrieve a data type from the registry by type information
      * 
      * \note If no data type matching the given type information exists in
      *   the data type registry, the returned data type will be invalid.
      */
    DataType getDataType(const std::type_info& typeInfo) const;
    
    /** \brief Retrieve a data type from the registry by type information
      *   (templated non-const version)
      * 
      * \note This version of the accessor deliberately attempts to add
      *   strong type information to an already registered array or message
      *   data type.
      */
    template <typename T> DataType getDataType();
    
    /** \brief Retrieve a data type from the registry by type information
      *   (templated const version)
      */
    template <typename T> DataType getDataType() const;
    
    /** \brief Write the data type registry to a stream
      */
    void write(std::ostream& stream) const;
    
    /** \brief Operator for retrieving a data type from the registry by
      *   identifier
      */
    DataType operator[](const std::string& identifier) const;
    
    /** \brief Operator for retrieving a data type from the registry by
      *   type information
      */
    DataType operator[](const std::type_info& typeInfo) const;
    
    /** \brief Add an array data type to the data type registry
      *   (variant-typed version)
      */ 
    ArrayDataType addArrayDataType(const DataType& elementType, size_t
      numElements = 0);
    
    /** \brief Add an array data type to the data type registry
      *   (strong-typed version templated on the array type)
      */ 
    template <class A> ArrayDataType addArrayDataType();
    
    /** \brief Add an array data type to the data type registry
      *   (strong-typed version templated on the array element type
      *   and the number of elements)
      */ 
    template <typename T, size_t N> ArrayDataType addArrayDataType();
    
    /** \brief Add a built-in data type to the data type registry
      */ 
    template <typename T> BuiltinDataType addBuiltinDataType(const
      std::string& identifier);
    
    /** \brief Add a message data type to the data type registry
      *   (variant-typed version taking an identifier and a sequence
      *   of message members)
      */ 
    MessageDataType addMessageDataType(const std::string&
      identifier, const std::vector<MessageMember>& members =
      std::vector<MessageMember>());
    
    /** \brief Add a message data type to the data type registry
      *   (variant-typed version taking an identifier and a definition)
      */ 
    MessageDataType addMessageDataType(const std::string&
      identifier, const std::string& definition);
    
    /** \brief Add a message data type to the data type registry
      *   (strong-typed version templated on the message type)
      */ 
    template <typename T> MessageDataType addMessageDataType();
    
    /** \brief Add a data type to the data type registry
      */
    void addDataType(const DataType& dataType);
    
    /** \brief Remove a data type from the data type registry
      */
    void removeDataType(const DataType& dataType);
    
    /** \brief Clear the data type registry
      */
    void clear();
    
  protected:
    /** \brief Type traits
      */
    struct TypeTraits {
      template <typename T, class Enable = void> struct DataTypeConstructor {
        static DataType create();
      };
        
      template <typename T> struct DataTypeConstructor<T, typename
          boost::enable_if<ArrayDataType::TypeTraits::IsArray<T> >::type> {
        static ArrayDataType create();
      };
      
      template <typename T> struct DataTypeConstructor<T, typename
          boost::enable_if<MessageDataType::TypeTraits::IsMessage<T> >::type> {
        static MessageDataType create();
      };      
    };
    
    /** \brief Data type implementation
      */
    class Impl {
    public:
      /** \brief Constructor
        */
      Impl();
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief The registered data types by identifier
        */
      boost::unordered_map<std::string, DataType> dataTypesByIdentifier;
      
      /** \brief The registered data types by type information
        */
      boost::unordered_multimap<const std::type_info*, DataType, TypeInfoHash>
        dataTypesByInfo;
   };
    
    /** \brief Declaration of the data type registry implementation pointer
      *   type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the data type registry implementation weak
      *   pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The data type registry's implementation
      */
    static ImplPtr impl;
  };
    
  std::ostream& operator<<(std::ostream& stream, const DataTypeRegistry&
    dataTypeRegistry);
};

#include <variant_topic_tools/DataTypeRegistry.tpp>

#endif
