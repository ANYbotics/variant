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

/** \file Subscriber.h
 * \brief Header file providing the Subscriber class interface
 */

#ifndef VARIANT_TOPIC_TOOLS_SUBSCRIBER_H
#define VARIANT_TOPIC_TOOLS_SUBSCRIBER_H

#include <ros/ros.h>

#include <variant_topic_tools/Forwards.h>
#include <variant_topic_tools/MessageSerializer.h>
#include <variant_topic_tools/MessageType.h>
#include <variant_topic_tools/MessageVariant.h>

namespace variant_topic_tools {
/** \brief Variant message subscriber
 */
class Subscriber {
  friend class MessageType;

 public:
  /** \brief Default constructor
   */
  Subscriber();

  /** \brief Copy constructor
   */
  Subscriber(const Subscriber& src);

  /** \brief Destructor
   */
  ~Subscriber();

  /** \brief Retrieve the topic this subscriber is subscribed to
   */
  std::string getTopic() const;

  /** \brief Retrieve this subscriber's number of publishers
   */
  size_t getNumPublishers() const;

  /** \brief Perform shutdown of the subscriber
   */
  void shutdown();

  /** \brief Void pointer conversion
   */
  inline operator void*() const { return (impl && impl->isValid()) ? (void*)1 : (void*)nullptr; };

  /** \brief Convert this variant message subscriber to a native ROS
   *   subscriber
   */
  operator ros::Subscriber() const;

 private:
  /** \brief Variant message subscriber implementation
   */
  class Impl {
   public:
    /** \brief Constructor
     */
    Impl(ros::NodeHandle& nodeHandle, const MessageType& type, const std::string& topic, size_t queueSize, SubscriberCallback callback);

    /** \brief Destructor
     */
    ~Impl();

    /** \brief True, if this subscriber implementation is valid
     */
    bool isValid() const;

    /** \brief Perform shutdown of the subscriber
     */
    void shutdown();

    /** \brief This subscriber's event callback
     */
    void eventCallback(const ros::MessageEvent<Message const>& event);

    /** \brief The message type associated with this subscriber
     */
    MessageType type;

    /** \brief The message data type associated with this subscriber
     */
    MessageDataType dataType;

    /** \brief The message serializer associated with this subscriber
     */
    MessageSerializer serializer;

    /** \brief The subscriber's callback
     */
    SubscriberCallback callback;

    /** \brief The ROS subscriber
     */
    ros::Subscriber subscriber;
  };

  /** \brief Declaration of the subscriber implementation pointer type
   */
  using ImplPtr = boost::shared_ptr<Impl>;

  /** \brief Declaration of the subscriber implementation weak pointer
   *   type
   */
  using ImplWPtr = boost::weak_ptr<Impl>;

  /** \brief The subscriber's implementation
   */
  ImplPtr impl;
};
}  // namespace variant_topic_tools

#endif
