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

/** \file MessageTypeTraits.h
  * \brief Header file providing the message type traits
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_TYPE_TRAITS_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_TYPE_TRAITS_H

#include <ros/message_traits.h>

namespace variant_topic_tools {
  namespace type_traits {
    template <typename T> struct IsMessage :
      public ros::message_traits::IsMessage<T> {
    };

    template <typename T> struct MessageType {
      typedef T ValueType;
    };
    
    template <typename T> struct ToMessageType {
      typedef T MessageType;
    };
  };
};

#endif
