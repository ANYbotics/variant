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

/** \file MessageHeader.h
 * \brief Header file providing the MessageHeader class interface
 */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_HEADER_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_HEADER_H

#include <ros/ros.h>

#include <variant_topic_tools/Forwards.h>

namespace variant_topic_tools {
/** \brief Variant message header
 */
class MessageHeader {
  friend class Message;
  friend class ros::serialization::PreDeserialize<Message>;

 public:
  /** \brief Default constructor
   */
  MessageHeader();

  /** \brief Copy constructor
   */
  MessageHeader(const MessageHeader& src);

  /** \brief Destructor
   */
  ~MessageHeader();

  /** \brief Set a field of the message header
   */
  void setField(const std::string& name, const std::string& value);

  /** \brief Retrieve a field of the message header
   */
  const std::string& getField(const std::string& name) const;

  /** \brief Set the name of the publishing node
   */
  void setPublisher(const std::string& publisher);

  /** \brief Retrieve the name of the publishing node
   */
  const std::string& getPublisher() const;

  /** \brief Set the message publishing topic
   */
  void setTopic(const std::string& topic);

  /** \brief Retrieve the message publishing topic
   */
  const std::string& getTopic() const;

  /** \brief Set to true to indicate that the message publication is
   *   latched
   */
  void setLatched(bool latched);

  /** \brief True, if the message publication is latched
   */
  bool isLatched() const;

  /** \brief True, if the message header contains a field with the
   *   specified name
   */
  bool hasField(const std::string& name) const;

  /** \brief Operator for retrieving a field of the message header
   *   (non-const version)
   */
  std::string& operator[](const std::string& name);

  /** \brief Operator for retrieving a field of the message header
   *   (const version)
   */
  const std::string& operator[](const std::string& name) const;

 protected:
  /** \brief Definition of the fields type
   */
  using Fields = std::map<std::string, std::string>;

  /** \brief Definition of the fields pointer type
   */
  using FieldsPtr = boost::shared_ptr<std::map<std::string, std::string>>;

  /** \brief The fields of the message header
   */
  FieldsPtr fields;

  /** \brief Constructor (overloaded version taking a fields pointer)
   */
  MessageHeader(FieldsPtr fields);
};
}  // namespace variant_topic_tools

#endif
