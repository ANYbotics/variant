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

/** \file MessageMemberPointer.h
 * \brief Header file providing the MessageMemberPointer class interface
 */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_MEMBER_POINTER_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_MEMBER_POINTER_H

#include <variant_topic_tools/DataTypeTraits.h>
#include <variant_topic_tools/MessageTypeTraits.h>
#include <variant_topic_tools/Pointer.h>

namespace variant_topic_tools {
/** \brief Shared pointer
 */
template <typename T, typename M>
class MessageMemberPointer : public Pointer<typename type_traits::DataType<M>::ValueType> {
 public:
  BOOST_STATIC_ASSERT(type_traits::IsMessage<T>::value);

  /** \brief Definition of the message value type
   */
  using ValueType = typename type_traits::MessageType<T>::ValueType;

  /** \brief Definition of the message member value type
   */
  using MemberValueType = typename type_traits::DataType<M>::ValueType;

  /** \brief Default constructor
   */
  MessageMemberPointer(ValueType* message = 0, size_t offset = 0);

  /** \brief Constructor (overloaded version taking a message pointer
   *   and an offset)
   */
  MessageMemberPointer(const Pointer<ValueType>& message, size_t offset);

  /** \brief Copy constructor
   */
  MessageMemberPointer(const MessageMemberPointer<T, M>& src);

  /** \brief Copy constructor (overloaded version taking a pointer)
   */
  MessageMemberPointer(const Pointer<MemberValueType>& src);

  /** \brief Destructor
   */
  ~MessageMemberPointer();

  /** \brief Set the message
   */
  void setMessage(const Pointer<ValueType>& message);

  /** \brief Retrieve the message
   */
  const Pointer<ValueType>& getMessage() const;

  /** \brief Set the message member offset
   */
  void setOffset(size_t offset);

  /** \brief Retrieve the message member offset
   */
  size_t getOffset() const;

 protected:
  /** \brief Message member pointer implementation
   */
  class Impl : public Pointer<MemberValueType>::ImplA {
   public:
    /** \brief Constructor
     */
    Impl(const Pointer<ValueType>& message = Pointer<ValueType>(), size_t offset = 0);

    /** \brief Destructor
     */
    virtual ~Impl();

    /** \brief Retrieve the stored pointer (implementation)
     */
    MemberValueType* get() const;

    /** \brief The shared message
     */
    Pointer<ValueType> message;

    /** \brief The member offset
     */
    size_t offset;
  };
};
}  // namespace variant_topic_tools

#include <variant_topic_tools/MessageMemberPointer.tpp>

#endif
