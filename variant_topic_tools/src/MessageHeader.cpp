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

#include "variant_topic_tools/MessageHeader.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageHeader::MessageHeader() : fields(new Fields()) {}

MessageHeader::MessageHeader(FieldsPtr fields) : fields(std::move(fields)) {}

MessageHeader::MessageHeader(const MessageHeader& src) = default;

MessageHeader::~MessageHeader() = default;

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageHeader::setField(const std::string& name, const std::string& value) {
  (*fields)[name] = value;
}

const std::string& MessageHeader::getField(const std::string& name) const {
  auto it = fields->find(name);

  if (it == fields->end()) {
    static std::string value = std::string();
    return value;
  } else {
    return it->second;
  }
}

void MessageHeader::setPublisher(const std::string& publisher) {
  (*fields)["callerid"] = publisher;
}

const std::string& MessageHeader::getPublisher() const {
  return getField("callerid");
}

void MessageHeader::setTopic(const std::string& topic) {
  (*fields)["topic"] = topic;
}

const std::string& MessageHeader::getTopic() const {
  return getField("topic");
}

void MessageHeader::setLatched(bool latched) {
  (*fields)["latching"] = latched ? "1" : "0";
}

bool MessageHeader::isLatched() const {
  return (getField("latching") == "1");
}

bool MessageHeader::hasField(const std::string& name) const {
  return (fields->find(name) != fields->end());
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

std::string& MessageHeader::operator[](const std::string& name) {
  return (*fields)[name];
}

const std::string& MessageHeader::operator[](const std::string& name) const {
  return getField(name);
}

}  // namespace variant_topic_tools
