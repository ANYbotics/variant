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

#include "variant_topic_tools/Message.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Message::Message(const MessageHeader& header, const MessageType& type, const
    std::vector<uint8_t>& data) :
  header(header),
  type(type),
  data(data) {
}

Message::Message(const Message& src) :
  header(src.header),
  type(src.type),
  data(src.data) {
}

Message::~Message() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void Message::setHeader(const MessageHeader& header) {
  this->header = header;
}

MessageHeader& Message::getHeader() {
  return header;
}

const MessageHeader& Message::getHeader() const {
  return header;
}

void Message::setType(const MessageType& type) {
  this->type = type;
}

MessageType& Message::getType() {
  return type;
}

const MessageType& Message::getType() const {
  return type;
}

void Message::setData(const std::vector<uint8_t>& data) {
  this->data = data;
}

std::vector<uint8_t>& Message::getData() {
  return data;
}

const std::vector<uint8_t>& Message::getData() const {
  return data;
}

uint32_t Message::getSize() const {
  return data.size();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

boost::shared_ptr<variant_msgs::Variant> Message::toVariant() const {
  boost::shared_ptr<variant_msgs::Variant> variant(
    new variant_msgs::Variant());
  
  variant->header.publisher = header.getPublisher();
  variant->header.topic = header.getTopic();
  variant->header.latched = header.isLatched();
  
  variant->type.data_type = type.getDataType();
  variant->type.md5_sum = type.getMD5Sum();
  variant->type.definition = type.getDefinition();
  
  variant->data = data;
  
  return variant;
}

}
