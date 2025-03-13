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

/** \file ArrayMemberPointer.h
 * \brief Header file providing the ArrayMemberPointer class interface
 */

#ifndef VARIANT_TOPIC_TOOLS_ARRAY_MEMBER_POINTER_H
#define VARIANT_TOPIC_TOOLS_ARRAY_MEMBER_POINTER_H

#include <variant_topic_tools/ArrayTypeTraits.h>
#include <variant_topic_tools/Pointer.h>

namespace variant_topic_tools {
/** \brief Shared pointer to an array member
 */
template <typename T>
class ArrayMemberPointer : public Pointer<typename type_traits::ArrayType<T>::MemberValueType> {
 public:
  BOOST_STATIC_ASSERT(type_traits::IsArray<T>::value);

  /** \brief Definition of the array value type
   */
  using ValueType = typename type_traits::ArrayType<T>::ValueType;

  /** \brief Definition of the array member type
   */
  using MemberValueType = typename type_traits::ArrayType<T>::MemberValueType;

  /** \brief Definition of the number of array members
   */
  static const size_t NumMembers = type_traits::ArrayType<T>::NumMembers;

  /** \brief Default constructor
   */
  ArrayMemberPointer(ValueType* array = 0, int index = 0);

  /** \brief Constructor (overloaded version taking an array pointer
   *   and an index)
   */
  ArrayMemberPointer(const Pointer<ValueType>& array, int index);

  /** \brief Copy constructor
   */
  ArrayMemberPointer(const ArrayMemberPointer<T>& src);

  /** \brief Copy constructor (overloaded version taking a pointer)
   */
  ArrayMemberPointer(const Pointer<MemberValueType>& src);

  /** \brief Destructor
   */
  ~ArrayMemberPointer();

  /** \brief Set the array
   */
  void setArray(const Pointer<ValueType>& array);

  /** \brief Retrieve the array
   */
  const Pointer<ValueType>& getArray() const;

  /** \brief Set the array member index
   */
  void setIndex(int index);

  /** \brief Retrieve the array member index
   */
  size_t getIndex() const;

 protected:
  /** \brief Array member pointer implementation
   */
  class Impl : public Pointer<MemberValueType>::ImplA {
   public:
    /** \brief Default constructor
     */
    Impl(const Pointer<ValueType>& array = Pointer<ValueType>(), size_t index = 0);

    /** \brief Destructor
     */
    virtual ~Impl();

    /** \brief Retrieve the stored pointer (implementation)
     */
    MemberValueType* get() const;

    /** \brief The shared array
     */
    Pointer<ValueType> array;

    /** \brief The member index
     */
    int index;
  };
};
}  // namespace variant_topic_tools

#include <variant_topic_tools/ArrayMemberPointer.tpp>

#endif
