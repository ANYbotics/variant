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

#include <variant_topic_tools/BuiltinTypeTraits.h>
#include <variant_topic_tools/DataType.h>

namespace variant_topic_tools {
/** \brief Built-in data type
 */
class BuiltinDataType : public DataType {
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
  ~BuiltinDataType() override;

  /** \brief Assignment operator
   */
  BuiltinDataType& operator=(const DataType& src) override;

  /** \brief True, if this built-in data type is numeric
   */
  bool isNumeric() const;

 protected:
  /** \brief Built-in data type implementation
   */
  class Impl : public DataType::Impl {
   public:
    /** \brief Constructor
     */
    Impl(std::string identifier);

    /** \brief Destructor
     */
    ~Impl() override;

    /** \brief Retrieve the identifier representing this data type
     *   (implementation)
     */
    const std::string& getIdentifier() const override;

    /** \brief True, if this built-in data type is numeric (abstract
     *   declaration)
     */
    virtual bool isNumeric() const = 0;

    /** \brief The identifier representing this built-in data type
     */
    std::string identifier;
  };

  /** \brief Built-in data type implementation (templated strong-typed
   *   version)
   */
  template <typename T>
  class ImplT : public Impl {
   public:
    BOOST_STATIC_ASSERT(type_traits::IsBuiltin<T>::value);

    /** \brief Constructor
     */
    ImplT(const std::string& identifier);

    /** \brief Destructor
     */
    ~ImplT() override;

    /** \brief Retrieve the type information associated with this data type
     *   (re-implementation)
     */
    const std::type_info& getTypeInfo() const override;

    /** \brief Retrieve the size of the instances of this data type
     *   (implementation)
     */
    size_t getSize() const override;

    /** \brief True, if this data type represents a fixed-size data type
     *   (implementation)
     */
    bool isFixedSize() const override;

    /** \brief True, if this data type represents a simple data type
     *   (implementation)
     */
    bool isSimple() const override;

    /** \brief True, if this built-in data type is numeric (implementation)
     */
    bool isNumeric() const override;

    /** \brief Create a serializer for this data type (re-implementation)
     */
    Serializer createSerializer(const DataType& type) const override;

    /** \brief Create a variant from this data type (re-implementation)
     */
    Variant createVariant(const DataType& type) const override;
  };

  /** \brief Create a built-in data type
   */
  template <typename T>
  static BuiltinDataType create(const std::string& identifier);
};
}  // namespace variant_topic_tools

#include <variant_topic_tools/BuiltinDataType.tpp>

#endif
