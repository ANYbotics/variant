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

/** \file MessageHeader.h
  * \brief Header file providing the MessageHeader class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_HEADER_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_HEADER_H

#include <ros/ros.h>

namespace variant_topic_tools {
  /** \brief Variant message header
    */
  class MessageHeader {
  friend class Message;
  public:
    /** \brief Default constructor
      */ 
    MessageHeader(const std::string& publisher = std::string(), const
      std::string& topic = std::string(), bool latched = false);
    
    /** \brief Copy constructor
      */ 
    MessageHeader(const MessageHeader& src);
    
    /** \brief Destructor
      */ 
    ~MessageHeader();

    /** \brief Set the name of the publishing node
      */ 
    void setPublisher(const std::string& publisher);
    
    /** \brief Retrieve the name of the publishing node
      */ 
    const std::string& getPublisher() const;
    
    /** \brief Set the message publishing topic
      */ 
    void setTopic(const std::string& topic);
    
    /** \brief Retrieve the message publishing topic
      */ 
    const std::string& getTopic() const;
    
    /** \brief Set to true to indicate that the message publication is
      *   latched
      */ 
    void setLatched(bool latched);
    
    /** \brief True, if the message publication is latched
      */ 
    bool isLatched() const;

  protected:
    /** \brief The message publisher
      */ 
    std::string publisher;
    
    /** \brief The message publishing topic
      */ 
    std::string topic;
    
    /** \brief True, if the message publication is latched
      */ 
    bool latched;
  };
};

#endif
