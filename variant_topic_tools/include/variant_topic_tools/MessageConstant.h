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

/** \file MessageConstant.h
 * \brief Header file providing the MessageConstant class interface
 */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_CONSTANT_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_CONSTANT_H

#include <variant_topic_tools/MessageMember.h>
#include <variant_topic_tools/Variant.h>

namespace variant_topic_tools {
/** \brief Constant message member
 */
class MessageConstant : public MessageMember {
  friend class MessageDataType;
  friend class MessageMember;

 public:
  /** \brief Default constructor
   */
  MessageConstant();

  /** \brief Copy constructor
   */
  MessageConstant(const MessageConstant& src);

  /** \brief Copy constructor (overloaded version taking a message member)
   */
  MessageConstant(const MessageMember& src);

  /** \brief Destructor
   */
  ~MessageConstant();

  /** \brief Retrieve the value of this message constant
   */
  const Variant& getValue() const;

 protected:
  /** \brief Message constant implementation
   */
  class Impl : public MessageMember::Impl {
   public:
    /** \brief Constructor
     */
    Impl(const std::string& name, const Variant& value);

    /** \brief Destructor
     */
    ~Impl() override;

    /** \brief Retrieve the type of this message member (implementation)
     */
    const DataType& getType() const override;

    /** \brief Write the message member to a stream (implementation)
     */
    void write(std::ostream& stream) const override;

    /** \brief The value of this message constant
     */
    Variant value;
  };

  /** \brief Constructor (overloaded version taking a name and a variant
   *   value)
   */
  MessageConstant(const std::string& name, const Variant& value);

  /** \brief Constructor (overloaded version taking a name, a data type,
   *   and a string value)
   */
  MessageConstant(const std::string& name, const DataType& type, const std::string& value);

  /** \brief Create a message constant
   */
  template <typename T>
  static MessageConstant create(const std::string& name, const T& value);
};
}  // namespace variant_topic_tools

#include <variant_topic_tools/MessageConstant.tpp>

#endif
