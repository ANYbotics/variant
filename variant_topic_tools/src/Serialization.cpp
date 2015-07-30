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

#include "variant_topic_tools/Serialization.h"

namespace ros {
  namespace serialization {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PreDeserialize<variant_topic_tools::Message>::notify(const
    PreDeserializeParams<variant_topic_tools::Message>& params) {
  variant_topic_tools::MessageHeader header;
  variant_topic_tools::MessageType type;
  
  header.setPublisher((*params.connection_header)["callerid"]);
  header.setTopic((*params.connection_header)["topic"]);
  header.setLatched((*params.connection_header)["latching"] == "1");
  
  type.setMD5Sum((*params.connection_header)["md5sum"]);
  type.setDataType((*params.connection_header)["type"]);
  type.setDefinition((*params.connection_header)["message_definition"]);

  params.message->setHeader(header);
  params.message->setType(type);
}

  }
}
