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

/** \file Publisher.h
  * \brief Header file providing the Publisher class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_PUBLISHER_H
#define VARIANT_TOPIC_TOOLS_PUBLISHER_H

#include <ros/ros.h>

#include <variant_topic_tools/Forwards.h>
#include <variant_topic_tools/MessageSerializer.h>
#include <variant_topic_tools/MessageType.h>
#include <variant_topic_tools/MessageVariant.h>

namespace variant_topic_tools {
  /** \brief Variant message publisher
    */
  class Publisher {
  friend class MessageType;
  public:
    /** \brief Default constructor
      */
    Publisher();
    
    /** \brief Copy constructor
      */
    Publisher(const Publisher& src);
    
    /** \brief Destructor
      */
    ~Publisher();
    
    /** \brief Retrieve the topic this publisher publishes on
      */
    std::string getTopic() const;
    
    /** \brief Retrieve this publisher's number of subscribers
      */
    size_t getNumSubscribers() const;
    
    /** \brief True, if the topic this publisher publishes on is latched
      */
    bool isLatched() const;
    
    /** \brief Perform shutdown of the publisher
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Publish a message variant on the topic associated with this
      *   publisher
      */ 
    void publish(const MessageVariant& variant);
  
    /** \brief Convert this variant message publisher to a native ROS
      *   publisher
      */ 
    operator ros::Publisher() const;
      
  private:
    /** \brief Variant message publisher implementation
      */
    class Impl {
    public:
      /** \brief Constructor
        */
      Impl(ros::NodeHandle& nodeHandle, const MessageType& type,
        const std::string& topic, size_t queueSize, bool latch = false,
        const ros::SubscriberStatusCallback& connectCallback =
        ros::SubscriberStatusCallback());
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief True, if this publisher implementation is valid
        */ 
      bool isValid() const;
      
      /** \brief Perform shutdown of the publisher
        */
      void shutdown();
            
      /** \brief Publish a message variant on the topic associated with this
        *   publisher
        */ 
      void publish(const MessageVariant& variant);
      
      /** \brief The message type associated with this publisher
        */ 
      MessageType type;
      
      /** \brief The message serializer associated with this publisher
        */ 
      MessageSerializer serializer;
      
      /** \brief The sequence number associated with this publisher
        */ 
      uint32_t sequenceNumber;
      
      /** \brief The ROS publisher
        */ 
      ros::Publisher publisher;
    };

    /** \brief Declaration of the publisher implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the publisher implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The publisher's implementation
      */
    ImplPtr impl;
  };
};

#endif
