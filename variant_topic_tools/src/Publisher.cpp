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

#include <ros/console.h>

#include "variant_topic_tools/DataTypeRegistry.h"
#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/Message.h"
#include "variant_topic_tools/Publisher.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Publisher::Publisher() = default;

Publisher::Publisher(const Publisher& src) = default;

Publisher::~Publisher() = default;

Publisher::Impl::Impl(ros::NodeHandle& nodeHandle, const MessageType& type, const std::string& topic, size_t queueSize, bool latch,
                      const ros::SubscriberStatusCallback& connectCallback)
    : type(type), sequenceNumber(0) {
  DataTypeRegistry registry;
  DataType dataType = registry.getDataType(type.getDataType());

  if (!dataType) {
    MessageDefinition definition(type);
    dataType = definition.getMessageDataType();
  }

  serializer = dataType.createSerializer();

  ros::AdvertiseOptions options(topic, queueSize, type.getMD5Sum(), type.getDataType(), type.getDefinition(), connectCallback);
  options.latch = latch;

  publisher = nodeHandle.advertise(options);
}

Publisher::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

Publisher::operator ros::Publisher() const {
  if (impl) {
    return impl->publisher;
  } else {
    return ros::Publisher();
  }
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string Publisher::getTopic() const {
  if (impl) {
    return impl->publisher.getTopic();
  } else {
    return std::string();
  }
}

size_t Publisher::getNumSubscribers() const {
  if (impl) {
    return impl->publisher.getNumSubscribers();
  } else {
    return 0;
  }
}

bool Publisher::isLatched() const {
  if (impl) {
    return impl->publisher.isLatched();
  } else {
    return false;
  }
}

bool Publisher::Impl::isValid() const {
  return type.isValid() && serializer && publisher;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Publisher::shutdown() {
  if (impl) {
    impl->shutdown();
  }
}

void Publisher::publish(const MessageVariant& variant) {
  if (impl && impl->isValid()) {
    impl->publish(variant);
  } else {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher");
  }
}

void Publisher::Impl::shutdown() {
  type = MessageType();
  serializer = MessageSerializer();
  sequenceNumber = 0;
  publisher = ros::Publisher();
}

void Publisher::Impl::publish(const MessageVariant& variant) {
  if (variant.getType().getIdentifier() == type.getDataType()) {
    Message message;
    MessageDataType dataType = variant.getType();

    if (dataType.hasHeader()) {
      variant["header/seq"] = sequenceNumber + 1;
    }

    message.setType(type);
    message.setSize(serializer.getSerializedLength(variant));
    ros::serialization::OStream stream(const_cast<uint8_t*>(message.getData().data()), message.getSize());

    serializer.serialize(stream, variant);
    publisher.publish(message);

    ++sequenceNumber;
  } else {
    throw MessageTypeMismatchException(type.getDataType(), variant.getType().getIdentifier());
  }
}

}  // namespace variant_topic_tools
