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

#include "variant_topic_tools/DataTypeRegistry.h"
#include "variant_topic_tools/Message.h"
#include "variant_topic_tools/MessageDefinition.h"
#include "variant_topic_tools/MessageSerializer.h"
#include "variant_topic_tools/MessageVariant.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Message::Message() {
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
  
  type.setMD5Sum(header["md5sum"]);
  type.setDataType(header["type"]);
  type.setDefinition(header["message_definition"]);
}

const MessageHeader& Message::getHeader() const {
  return header;
}

void Message::setType(const MessageType& type) {
  this->type = type;
  
  header["md5sum"] = type.getMD5Sum();
  header["type"] = type.getDataType();
  header["message_definition"] = type.getDefinition();
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

void Message::setSize(size_t size) {
  data.resize(size);
}

size_t Message::getSize() const {
  return data.size();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Message::serialize(const MessageVariant& variant) {
  MessageDataType dataType = variant.getType();
  MessageType type(dataType.getIdentifier(), dataType.getMD5Sum(),
    dataType.getDefinition());
  
  setType(type);
  MessageSerializer serializer = variant.createSerializer();
  data.resize(serializer.getSerializedLength(variant));
  ros::serialization::OStream stream(const_cast<uint8_t*>(
    data.data()), data.size());
  
  serializer.serialize(stream, variant);
}

void Message::deserialize(MessageVariant& variant) const {
  DataTypeRegistry registry;
  DataType dataType = registry.getDataType(type.getDataType());
  
  if (!dataType) {
    MessageDefinition definition(type);
    dataType = definition.getMessageDataType();
  }
  
  variant = dataType.createVariant();
  MessageSerializer serializer = variant.createSerializer();
  ros::serialization::IStream stream(const_cast<uint8_t*>(
    data.data()), data.size());
  
  serializer.deserialize(stream, variant);
}

boost::shared_ptr<variant_msgs::Variant> Message::toVariantMessage() const {
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
