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

MessageSerializer::MessageSerializer() = default;

MessageSerializer::MessageSerializer(const MessageFieldCollection<Serializer>& memberSerializers) {
  impl.reset(new ImplV(memberSerializers));
}

MessageSerializer::MessageSerializer(const MessageSerializer& src) = default;

MessageSerializer::MessageSerializer(const Serializer& src) : Serializer(src) {
  if (impl) {
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(impl));
  }
}

MessageSerializer::~MessageSerializer() = default;

MessageSerializer::Impl::Impl() = default;

MessageSerializer::Impl::~Impl() = default;

MessageSerializer::ImplV::ImplV(const MessageFieldCollection<Serializer>& memberSerializers) : memberSerializers(memberSerializers) {}

MessageSerializer::ImplV::~ImplV() = default;

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

size_t MessageSerializer::ImplV::getSerializedLength(const Variant& value) const {
  MessageVariant messageValue = value;
  size_t length = 0;

  for (size_t i = 0; i < messageValue.getNumMembers(); ++i) {
    length += memberSerializers[i].getValue().getSerializedLength(messageValue[i]);
  }

  return length;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageSerializer::ImplV::serialize(ros::serialization::OStream& stream, const Variant& value) {
  MessageVariant messageValue = value;

  for (size_t i = 0; i < messageValue.getNumMembers(); ++i) {
    memberSerializers[i].getValue().serialize(stream, messageValue[i]);
  }
}

void MessageSerializer::ImplV::deserialize(ros::serialization::IStream& stream, Variant& value) {
  MessageVariant messageValue = value;

  for (size_t i = 0; i < messageValue.getNumMembers(); ++i) {
    Variant member = messageValue[i];
    memberSerializers[i].getValue().deserialize(stream, member);
  }
}

}  // namespace variant_topic_tools
