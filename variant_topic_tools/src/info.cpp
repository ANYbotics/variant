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

#include <ros/ros.h>

#include <variant_topic_tools/Message.h>
#include <variant_topic_tools/MessageDefinition.h>

ros::NodeHandlePtr nodeHandle;

ros::Subscriber subscriber;
std::string subscriberTopic;
size_t subscriberQueueSize = 100;

variant_topic_tools::MessageDefinition messageDefinition;

void callback(const ros::MessageEvent<variant_topic_tools::Message>& messageEvent);

bool getTopicBase(const std::string& topic, std::string& topicBase) {
  std::string tmp = topic;
  int i = tmp.rfind('/');

  while ((!tmp.empty()) && (i >= (int)(tmp.size() - 1))) {
    tmp = tmp.substr(0, tmp.size() - 1);
    i = tmp.rfind('/');
  }

  if (tmp.empty()) {
    return false;
  }

  if (i < 0) {
    topicBase = tmp;
  } else {
    topicBase = tmp.substr(i + 1, tmp.size() - i - 1);
  }

  return true;
}

void subscribe() {
  subscriber = nodeHandle->subscribe(subscriberTopic, subscriberQueueSize, &callback);
}

void callback(const ros::MessageEvent<variant_topic_tools::Message>& messageEvent) {
  boost::shared_ptr<const variant_topic_tools::Message> message = messageEvent.getConstMessage();

  if (!messageDefinition.isValid()) {
    messageDefinition.setMessageType(message->getType());

    std::cout << "Topic: " << message->getHeader().getTopic() << "\n";
    std::cout << "Publisher: " << message->getHeader().getPublisher() << "\n";
    std::cout << "Latched: " << (message->getHeader().isLatched() ? "yes" : "no") << "\n";
    std::cout << "---\n";
    std::cout << "Type: " << message->getType().getDataType() << "\n";
    std::cout << "MD5 Sum: " << message->getType().getMD5Sum() << "\n";
    std::cout << "Definition:\n";
    std::cout << "---\n";
    std::cout << messageDefinition;
    std::cout << "---\n";

    ros::shutdown();
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("\nusage: info TOPIC\n\n");
    return 1;
  }

  if (!getTopicBase((argv[1]), subscriberTopic)) {
    ROS_ERROR("Failed to extract topic base from [%s]", argv[1]);
    return 1;
  }

  ros::init(argc, argv, subscriberTopic + "_info", ros::init_options::AnonymousName);

  subscriberTopic = argv[1];

  nodeHandle.reset(new ros::NodeHandle("~"));

  subscribe();
  ros::spin();

  return 0;
}
