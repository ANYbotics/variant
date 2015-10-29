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

#include "variant_topic_tools/MessageSerializer.h"
#include "variant_topic_tools/MessageVariant.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageSerializer::MessageSerializer() {
}

MessageSerializer::MessageSerializer(const std::vector<MessageVariable>&
    members) {
  impl.reset(new ImplV(members));
}

MessageSerializer::MessageSerializer(const MessageSerializer& src) :
  Serializer(src) {
}

MessageSerializer::MessageSerializer(const Serializer& src) :
  Serializer(src) {
  if (impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(impl));
}

MessageSerializer::~MessageSerializer() {
}

MessageSerializer::Impl::Impl() {
}

MessageSerializer::Impl::~Impl() {
}

MessageSerializer::ImplV::ImplV(const std::vector<MessageVariable>& members) {
  for (size_t i = 0; i < members.size(); ++i)
    memberSerializers.appendField(members[i].getName(),
      members[i].getType().createSerializer());
}

MessageSerializer::ImplV::~ImplV() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageSerializer::ImplV::serialize(ros::serialization::OStream& stream,
    const Variant& value) {
  MessageVariant messageValue = value;
  
  for (size_t i = 0; i < messageValue.getNumMembers(); ++i)
    memberSerializers[i].getValue().serialize(stream, messageValue[i]);
}

void MessageSerializer::ImplV::deserialize(ros::serialization::IStream& stream,
    Variant& value) {
  MessageVariant messageValue = value;

  for (size_t i = 0; i < messageValue.getNumMembers(); ++i) {
    Variant member = messageValue[i];
    memberSerializers[i].getValue().deserialize(stream, member);
  }
}

void MessageSerializer::ImplV::advance(ros::serialization::IStream& stream,
    const Variant& value) {
  MessageVariant messageValue = value;

  for (size_t i = 0; i < messageValue.getNumMembers(); ++i)
    memberSerializers[i].getValue().advance(stream, messageValue[i]);
}

}
