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

/** \file MessageField.h
 * \brief Header file providing the MessageField class interface
 */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_FIELD_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_FIELD_H

#include <boost/type_traits.hpp>

#include <ros/ros.h>

#include <variant_topic_tools/MessageFieldCollection.h>
#include <variant_topic_tools/MessageFieldTypeTraits.h>
#include <variant_topic_tools/MessageType.h>

namespace variant_topic_tools {
/** \brief Message field template
 */
template <typename T>
class MessageField : public MessageFieldCollection<T> {
 public:
  /** \brief Default constructor
   */
  MessageField(const std::string& name = std::string(), const T& value = T());

  /** \brief Copy constructor
   */
  MessageField(const MessageField<T>& src);

  /** \brief Destructor
   */
  virtual ~MessageField();

  /** \brief Set the name of the message field
   */
  void setName(const std::string& name);

  /** \brief Retrieve the name of the message field
   */
  const std::string& getName() const;

  /** \brief Set the value of the message field
   */
  void setValue(const T& value);

  /** \brief Retrieve the value of the message field (non-const version)
   */
  T& getValue();

  /** \brief Retrieve the value of the message field (const version)
   */
  const T& getValue() const;

  /** \brief True, if this message field is valid
   */
  bool isValid() const;

  /** \brief Clear the message field
   */
  void clear();

  /** \brief Write the message field to a stream
   */
  void write(std::ostream& stream, const std::string& indent = std::string()) const;

  /** \brief True, if this message field is equal to another message field
   */
  bool operator==(const MessageField<T>& field) const;

  /** \brief True, if this message field is not equal to another message
   *   field
   */
  bool operator!=(const MessageField<T>& field) const;

 protected:
  /** \brief The name of this message field
   */
  std::string name;

  /** \brief The value of this message field
   */
  T value;

  /** \brief True, if the message field is valid (overloaded version
   *   taking a field value which can be validated)
   */
  template <typename U>
  static bool isValid(const U& value, typename boost::enable_if_c<MessageFieldTypeTraits::HasIsValid<U>::value>::type* /*unused*/ = 0);

  /** \brief True, if the message field is valid (overloaded version
   *   taking a field value which cannot be validated)
   */
  template <typename U>
  static bool isValid(const U& value, typename boost::disable_if_c<MessageFieldTypeTraits::HasIsValid<U>::value>::type* /*unused*/ = 0);

  /** \brief Write a message field value to a stream (overloaded version
   *   taking a stream-writable value)
   */
  template <typename U>
  static void writeValue(std::ostream& stream, const U& value,
                         typename boost::enable_if<boost::has_left_shift<std::ostream, const U&> >::type* /*unused*/ = 0);

  /** \brief Write a message field value to a stream (overloaded version
   *   taking a non-stream-writable value)
   */
  template <typename U>
  static void writeValue(std::ostream& stream, const U& value,
                         typename boost::disable_if<boost::has_left_shift<std::ostream, const U&> >::type* /*unused*/ = 0);
};

/** \brief Operator for writing the message field to a stream
 */
template <typename T>
std::ostream& operator<<(std::ostream& stream, const MessageField<T>& messageField);
}  // namespace variant_topic_tools

#include <variant_topic_tools/MessageField.tpp>

#endif
