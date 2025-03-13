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

namespace ros {
namespace serialization {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class Stream>
void Serializer<variant_topic_tools::Message>::read(Stream& stream, variant_topic_tools::Message& message) {
  message.read(stream);
}

template <class Stream>
void Serializer<variant_topic_tools::Message>::write(Stream& stream, const variant_topic_tools::Message& message) {
  message.write(stream);
}

uint32_t Serializer<variant_topic_tools::Message>::serializedLength(const variant_topic_tools::Message& message) {
  return message.getSize();
}

}  // namespace serialization
}  // namespace ros
