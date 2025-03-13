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

#include "variant_topic_tools/MessageTraits.h"

namespace ros {
namespace message_traits {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

const char* MD5Sum<variant_topic_tools::Message>::value(const variant_topic_tools::Message& message) {
  return message.getType().getMD5Sum().c_str();
}

const char* MD5Sum<variant_topic_tools::Message>::value() {
  return "*";
}

const char* DataType<variant_topic_tools::Message>::value(const variant_topic_tools::Message& message) {
  return message.getType().getDataType().c_str();
}

const char* DataType<variant_topic_tools::Message>::value() {
  return "*";
}

const char* Definition<variant_topic_tools::Message>::value(const variant_topic_tools::Message& message) {
  return message.getType().getDefinition().c_str();
}

}  // namespace message_traits
}  // namespace ros
