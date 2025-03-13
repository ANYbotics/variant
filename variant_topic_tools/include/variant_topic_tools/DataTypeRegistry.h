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

#include <boost/type_traits.hpp>
#include <boost/unordered_map.hpp>

#include <variant_topic_tools/DataTypeTraits.h>
#include <variant_topic_tools/Forwards.h>
#include <variant_topic_tools/MessageFieldCollection.h>
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
   *   (non-const version)
   *
   * \note This version of the accessor deliberately attempts to add
   *   an array data type for an already registered message or built-in
   *   data type. If no data type matching the given identifier or array
   *   member type identifier exists in the data type registry, the
   *   returned data type will be invalid.
   */
  DataType getDataType(const std::string& identifier);

  /** \brief Retrieve a data type from the registry by identifier
   *   (const version)
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
  template <typename T>
  DataType getDataType();

  /** \brief Retrieve a data type from the registry by type information
   *   (templated const version)
   */
  template <typename T>
  DataType getDataType() const;

  /** \brief Write the data type registry to a stream
   */
  static void write(std::ostream& stream);

  /** \brief Operator for retrieving a data type from the registry by
   *   identifier (non-const version)
   */
  DataType operator[](const std::string& identifier);

  /** \brief Operator for retrieving a data type from the registry by
   *   identifier (const version)
   */
  DataType operator[](const std::string& identifier) const;

  /** \brief Operator for retrieving a data type from the registry by
   *   type information
   */
  DataType operator[](const std::type_info& typeInfo) const;

  /** \brief Add an array data type to the data type registry
   *   (variant-typed version)
   */
  static ArrayDataType addArrayDataType(const DataType& memberType, size_t numMembers = 0);

  /** \brief Add an array data type to the data type registry
   *   (strong-typed version templated on the array type)
   */
  template <typename T>
  ArrayDataType addArrayDataType();

  /** \brief Add an array data type to the data type registry
   *   (strong-typed version templated on the array member type
   *   and the number of members)
   */
  template <typename T, size_t N>
  ArrayDataType addArrayDataType();

  /** \brief Add a built-in data type to the data type registry
   */
  template <typename T>
  BuiltinDataType addBuiltinDataType(const std::string& identifier);

  /** \brief Add a message data type to the data type registry
   *   (variant-typed version taking an identifier, a collection
   *   of constant message members, and a collection of variable
   *   message members)
   */
  static MessageDataType addMessageDataType(
      const std::string& identifier,
      const MessageFieldCollection<MessageConstant>& constantMembers = MessageFieldCollection<MessageConstant>(),
      const MessageFieldCollection<MessageVariable>& variableMembers = MessageFieldCollection<MessageVariable>());

  /** \brief Add a message data type to the data type registry
   *   (variant-typed version taking an identifier and a definition)
   */
  static MessageDataType addMessageDataType(const std::string& identifier, const std::string& definition);

  /** \brief Add a message data type to the data type registry
   *   (strong-typed version templated on the message type)
   */
  template <typename T>
  MessageDataType addMessageDataType();

  /** \brief Add a data type to the data type registry
   */
  static void addDataType(const DataType& dataType);

  /** \brief Add a data type to the data type registry (templated
   *   version)
   */
  template <typename T>
  void addDataType();

  /** \brief Remove a data type from the data type registry
   */
  static void removeDataType(const DataType& dataType);

  /** \brief Clear the data type registry
   */
  static void clear();

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
    ~Impl();

    /** \brief The registered data types by identifier
     */
    boost::unordered_map<std::string, DataType> dataTypesByIdentifier;

    /** \brief The registered data types by type information
     */
    boost::unordered_multimap<const std::type_info*, DataType, TypeInfoHash> dataTypesByInfo;
  };

  /** \brief Declaration of the data type registry implementation pointer
   *   type
   */
  using ImplPtr = boost::shared_ptr<Impl>;

  /** \brief Declaration of the data type registry implementation weak
   *   pointer type
   */
  using ImplWPtr = boost::weak_ptr<Impl>;

  /** \brief The data type registry's implementation
   */
  static ImplPtr impl;

  /** \brief Create a data type (overloaded version creating an invalid
   *   built-in data type)
   */
  template <typename T>
  static BuiltinDataType createDataType(typename boost::enable_if<type_traits::IsBuiltin<T> >::type* /*unused*/ = 0);

  /** \brief Create a data type (overloaded version creating an array
   *   data type)
   */
  template <typename T>
  static ArrayDataType createDataType(typename boost::enable_if<type_traits::IsArray<T> >::type* /*unused*/ = 0);

  /** \brief Create a data type (overloaded version creating a message
   *   data type)
   */
  template <typename T>
  static MessageDataType createDataType(typename boost::enable_if<type_traits::IsMessage<T> >::type* /*unused*/ = 0);
};

/** \brief Operator for writing the data type registry to a stream
 */
std::ostream& operator<<(std::ostream& stream, const DataTypeRegistry& dataTypeRegistry);
}  // namespace variant_topic_tools

#include <variant_topic_tools/DataTypeRegistry.tpp>

#endif
