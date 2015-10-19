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

/** \file MessageStream.h
  * \brief Header file providing the MessageStream class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_STREAM_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_STREAM_H

#include <typeinfo>
#include <vector>

#include <variant_topic_tools/Forwards.h>

namespace variant_topic_tools {
  /** \brief Message output stream
    * 
    * This stream implementation determines the offsets of the members
    * of a message.
    */
  class MessageStream {
  public:
    /** \brief Constructor
      */ 
    MessageStream(const uint8_t* data);
    
    /** \brief Constructor (templated version)
      */ 
    template <typename T> MessageStream(const T& data);
    
    /** \brief Copy constructor
      */ 
    MessageStream(const MessageStream& src);
    
    /** \brief Destructor
      */ 
    ~MessageStream();

    /** \brief Access the message stream's data
      */
    const uint8_t* getData() const;

    /** \brief Access the number of streamed message members
      */
    size_t getNumMembers() const;
    
    /** \brief Access the type information of the streamed message member
      *   with the specified index
      */
    const std::type_info& getMemberTypeInfo(size_t index) const;
    
    /** \brief Access the offset of the streamed message member with the
      *   specified index
      */
    size_t getMemberOffset(size_t index) const;
    
    /** \brief Advance the message stream
      */
    template <typename T> void next(const T& member);
    
    /** \brief Message stream output operator
      */
    template <typename T> MessageStream& operator<<(const T& member);
    
  protected:
    /** \brief The message stream's data
      */
    const uint8_t* data;
    
    /** \brief The type information of the streamed message members
      */
    std::vector<const std::type_info*> memberTypeInfo;
    
    /** \brief The offsets of the streamed message members
      */
    std::vector<size_t> memberOffsets;
  };
};

#include <variant_topic_tools/MessageStream.tpp>

#endif
