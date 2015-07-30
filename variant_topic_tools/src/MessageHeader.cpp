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

#include "variant_topic_tools/MessageHeader.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageHeader::MessageHeader(const std::string& publisher, const std::string&
    topic, bool latched) :
  publisher(publisher),
  topic(topic),
  latched(latched) {
}

MessageHeader::MessageHeader(const MessageHeader& src) :
  publisher(src.publisher),
  topic(src.topic),
  latched(src.latched) {
}

MessageHeader::~MessageHeader() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageHeader::setPublisher(const std::string& publisher) {
  this->publisher = publisher;
}

const std::string& MessageHeader::getPublisher() const {
  return publisher;
}

void MessageHeader::setTopic(const std::string& topic) {
  this->topic = topic;
}

const std::string& MessageHeader::getTopic() const {
  return topic;
}

bool MessageHeader::isLatched() const {
  return latched;
}


void MessageHeader::setLatched(bool latched) {
  this->latched = latched;
}

}
