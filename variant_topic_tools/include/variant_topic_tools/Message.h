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

/** \file Message.h
  * \brief Header file providing the Message class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_H

#include <vector>

#include <ros/ros.h>

#include <variant_msgs/Variant.h>

#include <variant_topic_tools/Forwards.h>
#include <variant_topic_tools/MessageHeader.h>
#include <variant_topic_tools/MessageType.h>

namespace variant_topic_tools {
  /** \brief Generic message type
    * 
    * This generic message type can be used to subscribe to any topic.
    * It is heavily inspired by the ShapeShifter message type provided
    * in the topic_tools package.
    */
  class Message {
  public:
    /** \brief Default constructor
      */ 
    Message();
    
    /** \brief Constructor (templated version taking a message
      */ 
    template <typename T> Message(const T& message, const MessageHeader&
      header = MessageHeader());
    
    /** \brief Copy constructor
      */ 
    Message(const Message& src);
    
    /** \brief Destructor
      */ 
    ~Message();

    /** \brief Set the message header
      * 
      * \note This will modify the message type from the fields of the
      *   specified header.
      */
    void setHeader(const MessageHeader& header);
    
    /** \brief Retrieve the message header
      */
    const MessageHeader& getHeader() const;
    
    /** \brief Set the message type
      * 
      * \note This will modify the message header from the members of the
      *   specified type.
      */
    void setType(const MessageType& type);
    
    /** \brief Retrieve the message type
      */
    const MessageType& getType() const;

    /** \brief Set the message data
      */
    void setData(const std::vector<uint8_t>& data);
    
    /** \brief Retrieve the message data (non-const version)
      */
    std::vector<uint8_t>& getData();
    
    /** \brief Retrieve the message data (const version)
      */
    const std::vector<uint8_t>& getData() const;
    
    /** \brief Set the message size
      */
    void setSize(size_t size);

    /** \brief Retrieve the message size
      */
    size_t getSize() const;

    /** \brief Morph the message
      */
    template <typename T> void morph();

    /** \brief Attempt to serialize this message from a variant
      */
    void serialize(const MessageVariant& variant);
      
    /** \brief Attempt to deserialize this message into a variant
      */
    void deserialize(MessageVariant& variant) const;
    
    /** \brief Attempt to convert the message to a variant message
      */
    boost::shared_ptr<variant_msgs::Variant> toVariantMessage() const;
      
    /** \brief Attempt to convert the message to a strong-typed message
      */
    template <typename T> boost::shared_ptr<T> toMessage() const;

    /** \brief Read serialized message contents from stream
      */ 
    template <typename Stream> void read(Stream& stream);

    /** \brief Write serialized message contents to stream
      */
    template <typename Stream> void write(Stream& stream) const;

  protected:
    /** \brief The header of this message
      */ 
    MessageHeader header;
    
    /** \brief The type of this message
      */ 
    MessageType type;
    
    /** \brief The data of this message
      */ 
    std::vector<uint8_t> data;
  };
};

#include <variant_topic_tools/Message.tpp>

#include <variant_topic_tools/MessageTraits.h>
#include <variant_topic_tools/Serialization.h>

#endif
