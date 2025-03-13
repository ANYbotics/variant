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

#include <variant_topic_tools/Publisher.h>

ros::NodeHandlePtr nodeHandle;

variant_topic_tools::Publisher publisher;
std::string publisherTopic;
std::string publisherType;
double publisherRate = 0.0;
size_t publisherQueueSize = 100;
ros::Timer publisherTimer;

variant_topic_tools::MessageDefinition messageDefinition;
variant_topic_tools::MessageType messageType;

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

void publishOnce() {
  if (!messageDefinition.isValid()) {
    messageDefinition.load(publisherType);
  }

  if (!messageType.isValid()) {
    messageType = messageDefinition.getMessageDataType();
  }

  if (!publisher) {
    publisher = messageType.advertise(*nodeHandle, publisherTopic, publisherQueueSize, true);
  }

  variant_topic_tools::MessageVariant variant = messageDefinition.getMessageDataType().createVariant();

  publisher.publish(variant);
}

void publish(const ros::TimerEvent& /*event*/) {
  publishOnce();
}

int main(int argc, char** argv) {
  if (argc < 3) {
    printf("\nusage: echo TOPIC TYPE [RATE]\n\n");
    return 1;
  }

  if (!getTopicBase((argv[1]), publisherTopic)) {
    ROS_ERROR("Failed to extract topic base from [%s]", argv[1]);
    return 1;
  }

  ros::init(argc, argv, publisherTopic + "_publish", ros::init_options::AnonymousName);

  nodeHandle.reset(new ros::NodeHandle("~"));

  publisherTopic = argv[1];
  publisherType = argv[2];
  if (argc > 3) {
    publisherRate = boost::lexical_cast<double>(argv[3]);
  }

  if (publisherRate > 0.0) {
    publisherTimer = nodeHandle->createTimer(ros::Rate(publisherRate).expectedCycleTime(), &publish);
  } else {
    publishOnce();
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::waitForShutdown();

  publisherTimer.stop();

  return 0;
}
