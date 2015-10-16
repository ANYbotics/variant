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

#include <algorithm>

#include <variant_topic_tools/Exceptions.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T> Message::Message(const T& message, const MessageHeader&
    header) :
  header(header),
  type(MessageType::template create<T>()),
  data(ros::serialization::serializationLength(message)) {
  BOOST_STATIC_ASSERT(ros::message_traits::IsMessage<T>::value);

  ros::serialization::OStream stream(const_cast<uint8_t*>(data.data()),
    data.size());
  ros::serialization::serialize(stream, message);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> void Message::morph() {
  this->setType(MessageType::template create<T>());
}

template <typename T> boost::shared_ptr<T> Message::toInvariant() const {
  if (ros::message_traits::template datatype<T>() != type.getDataType())
    throw DataTypeMismatchException(ros::message_traits::template
      datatype<T>(), type.getDataType());
  
  if ((type.getMD5Sum() != "*") &&
      (ros::message_traits::template md5sum<T>() != type.getMD5Sum()))
    throw MD5SumMismatchException(ros::message_traits::template 
      md5sum<T>(), type.getMD5Sum());
  
  boost::shared_ptr<T> invariant(new T());

  ros::serialization::IStream stream(const_cast<uint8_t*>(data.data()),
    data.size());
  ros::serialization::deserialize(stream, *invariant);

  return invariant;
}

template <class Stream> void Message::read(Stream& stream) {
  size_t size = stream.getLength();
  uint8_t* data = stream.getData();
  
  if (size > this->data.capacity())
    this->data.reserve(size);
  
  std::copy(&data[0], &data[size], std::back_inserter(this->data));
}

template <class Stream> void Message::write(Stream& stream) const {
  if (data.size() > 0)
    std::copy(data.begin(), data.end(), stream.advance(data.size()));
}

}
