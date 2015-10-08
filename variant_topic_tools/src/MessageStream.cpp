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
#include "variant_topic_tools/MessageStream.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageStream::MessageStream(const uint8_t* data) :
  data(data) {
}

MessageStream::MessageStream(const MessageStream& src) :
  data(src.data),
  memberTypes(src.memberTypes),
  memberOffsets(src.memberOffsets) {
}

MessageStream::~MessageStream() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const uint8_t* MessageStream::getData() const {
  return data;
}

const std::vector<DataType>& MessageStream::getMemberTypes() const {
  return memberTypes;
}

const std::vector<size_t>& MessageStream::getMemberOffsets() const {
  return memberOffsets;
}

}
