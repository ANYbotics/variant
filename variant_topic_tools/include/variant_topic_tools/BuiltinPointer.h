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

/** \file BuiltinPointer.h
 * \brief Header file providing the BuiltinPointer class interface
 */

#ifndef VARIANT_TOPIC_TOOLS_BUILTIN_POINTER_H
#define VARIANT_TOPIC_TOOLS_BUILTIN_POINTER_H

#include <variant_topic_tools/BuiltinTypeTraits.h>
#include <variant_topic_tools/Pointer.h>

namespace variant_topic_tools {
/** \brief Shared pointer to a built-in variable
 */
template <typename T>
class BuiltinPointer : public Pointer<typename type_traits::BuiltinType<T>::ValueType> {
 public:
  BOOST_STATIC_ASSERT(type_traits::IsBuiltin<T>::value);

  /** \brief Definition of the built-in value type
   */
  using ValueType = typename type_traits::BuiltinType<T>::ValueType;

  /** \brief Default constructor
   */
  BuiltinPointer(ValueType* builtin = 0);

  /** \brief Constructor (overloaded version taking a built-in pointer
   */
  BuiltinPointer(const boost::shared_ptr<ValueType>& builtin);

  /** \brief Copy constructor
   */
  BuiltinPointer(const BuiltinPointer<T>& src);

  /** \brief Copy constructor (overloaded version taking a pointer)
   */
  BuiltinPointer(const Pointer<ValueType>& src);

  /** \brief Destructor
   */
  ~BuiltinPointer();

 protected:
  /** \brief Built-in pointer implementation
   */
  class Impl : public Pointer<ValueType>::ImplA {
   public:
    /** \brief Constructor
     */
    Impl(const boost::shared_ptr<ValueType>& builtin = boost::shared_ptr<ValueType>());

    /** \brief Destructor
     */
    virtual ~Impl();

    /** \brief Retrieve the stored pointer (implementation)
     */
    ValueType* get() const;

    boost::shared_ptr<ValueType> builtin;
  };
};
}  // namespace variant_topic_tools

#include <variant_topic_tools/BuiltinPointer.tpp>

#endif
