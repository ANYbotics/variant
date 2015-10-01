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

#include <variant_topic_tools/DataTypeRegistry.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T> MessageStream::MessageStream(const T& data) :
  data(reinterpret_cast<const uint8_t*>(&data)) {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> void MessageStream::next(const T& member) {
  DataTypeRegistry registry;
  
  this->memberTypes.push_back(registry.getDataType<T>());
  this->memberOffsets.push_back(reinterpret_cast<size_t>(&member)-
    reinterpret_cast<size_t>(data));
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

template <typename T> MessageStream& MessageStream::operator<<(
    const T& member) {
  this->next(member);
  return *this;
}

}
