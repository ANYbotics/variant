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
/* Methods                                                                   */
/*****************************************************************************/

template <class Invariant> void Message::morph() {
  MessageType type(
    ros::message_traits::template datatype<Invariant>(),
    ros::message_traits::template md5sum<Invariant>(),
    ros::message_traits::template definition<Invariant>()
  );
  
  this->setType(type);
}

template <class Invariant> boost::shared_ptr<Invariant> Message::toInvariant()
    const {
  if (ros::message_traits::template datatype<Invariant>() != type.dataType)
    throw DataTypeMismatchException(ros::message_traits::template
      datatype<Invariant>(), type.dataType);
  
  if ((type.md5Sum != "*") &&
      (ros::message_traits::template md5sum<Invariant>() != type.md5Sum))
    throw MD5SumMismatchException(ros::message_traits::template 
      md5sum<Invariant>(), type.md5Sum);
  
  boost::shared_ptr<Invariant> invariant(new Invariant());

  ros::serialization::IStream stream(data.data(), data.size());
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
