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

#include <utility>

#include "variant_topic_tools/DataTypeRegistry.h"
#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/Message.h"
#include "variant_topic_tools/Subscriber.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

Subscriber::Subscriber() = default;

Subscriber::Subscriber(const Subscriber& src) = default;

Subscriber::~Subscriber() = default;

Subscriber::Impl::Impl(ros::NodeHandle& nodeHandle, const MessageType& type, const std::string& topic, size_t queueSize,
                       SubscriberCallback callback)
    : type(type), callback(std::move(callback)) {
  subscriber = nodeHandle.subscribe(topic, queueSize, &Subscriber::Impl::eventCallback, this);
}

Subscriber::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

Subscriber::operator ros::Subscriber() const {
  if (impl) {
    return impl->subscriber;
  } else {
    return ros::Subscriber();
  }
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string Subscriber::getTopic() const {
  if (impl) {
    return impl->subscriber.getTopic();
  } else {
    return std::string();
  }
}

size_t Subscriber::getNumPublishers() const {
  if (impl) {
    return impl->subscriber.getNumPublishers();
  } else {
    return 0;
  }
}

bool Subscriber::Impl::isValid() const {
  return subscriber != nullptr;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void Subscriber::shutdown() {
  if (impl) {
    impl->shutdown();
  }
}

void Subscriber::Impl::shutdown() {
  subscriber = ros::Subscriber();

  type = MessageType();
  dataType = DataType();
  serializer = MessageSerializer();
}

void Subscriber::Impl::eventCallback(const ros::MessageEvent<Message const>& event) {
  boost::shared_ptr<const Message> message = event.getConstMessage();

  if (!type.isValid()) {
    type = message->getType();
  }

  if (message->getType().getDataType() == type.getDataType()) {
    if ((type.getMD5Sum() == "*") || (message->getType().getMD5Sum() == "*") || (message->getType().getMD5Sum() == type.getMD5Sum())) {
      if (!dataType) {
        DataTypeRegistry registry;
        dataType = registry.getDataType(type.getDataType());

        if (!dataType) {
          type = message->getType();

          MessageDefinition definition(type);
          dataType = definition.getMessageDataType();
        }
      }

      if (dataType && !serializer) {
        serializer = dataType.createSerializer();
      }

      if (serializer) {
        MessageVariant variant = dataType.createVariant();
        ros::serialization::IStream stream(const_cast<uint8_t*>(message->getData().data()), message->getSize());

        serializer.deserialize(stream, variant);

        callback(variant, event.getReceiptTime());
      }
    } else {
      throw MD5SumMismatchException(type.getMD5Sum(), message->getType().getMD5Sum());
    }
  } else {
    throw MessageTypeMismatchException(type.getDataType(), message->getType().getDataType());
  }
}

}  // namespace variant_topic_tools
