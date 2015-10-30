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

ros::NodeHandlePtr nodeHandle;

ros::Subscriber subscriber;
std::string subscriberTopic;
size_t subscriberQueueSize = 100;
ros::TransportHints subscriberTransportHints;

ros::Publisher publisher;
std::string publisherTopic;
size_t publisherQueueSize = 100;

bool lazy = false;

void connectCallback(const ros::SingleSubscriberPublisher&);
void callback(const ros::MessageEvent<variant_topic_tools::Message>&
  messageEvent);

bool getTopicBase(const std::string& topic, std::string& topicBase) {
  std::string tmp = topic;
  int i = tmp.rfind('/');

  while( (tmp.size() > 0) && (i >= (int)(tmp.size()-1))) {
    tmp = tmp.substr(0,tmp.size()-1);
    i = tmp.rfind('/');
  }

  if (tmp.size() == 0)
    return false;

  if(i < 0)
    topicBase = tmp;
  else
    topicBase = tmp.substr(i+1, tmp.size()-i-1);

  return true;
}

void subscribe() {
  subscriber = nodeHandle->subscribe(subscriberTopic, subscriberQueueSize,
    &callback, subscriberTransportHints);
}

void connectCallback(const ros::SingleSubscriberPublisher&) {
  if(lazy && !subscriber)
    subscribe();
}

void callback(const ros::MessageEvent<variant_topic_tools::Message>&
    messageEvent) {
  boost::shared_ptr<const variant_topic_tools::Message> message =
    messageEvent.getConstMessage();
  boost::shared_ptr<const ros::M_string> connectionHeader =
    messageEvent.getConnectionHeaderPtr();

  if (!publisher) {
    bool latch = false;
    
    if (connectionHeader) {
      ros::M_string::const_iterator it = connectionHeader->find("latching");
      if ((it != connectionHeader->end()) && (it->second == "1"))
        latch = true;
    }
    
    ros::AdvertiseOptions options(publisherTopic, publisherQueueSize,
      message->getType().getMD5Sum(), message->getType().getDataType(),
      message->getType().getDefinition(), connectCallback);
    options.latch = latch;
    
    publisher = nodeHandle->advertise<variant_msgs::Variant>(publisherTopic,
      publisherQueueSize, connectCallback, ros::SubscriberStatusCallback(),
      ros::VoidConstPtr(), latch);
  }

  if(!lazy || publisher.getNumSubscribers()) {
    boost::shared_ptr<const variant_msgs::Variant> variantMessage =
      message->toVariantMessage();
    publisher.publish(variantMessage);
  }
  else
    subscriber = ros::Subscriber();
}

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("\nusage: relay TOPIC [VARIANT_TOPIC]\n\n");
    return 1;
  }
  
  if (!getTopicBase((argv[1]), publisherTopic)) {
    ROS_ERROR("Failed to extract topic base from [%s]", argv[1]);
    return 1;
  }
  
  ros::init(argc, argv, publisherTopic+"_relay",
    ros::init_options::AnonymousName);
  
  if (argc == 2)
    publisherTopic = std::string(argv[1])+"_relay";
  else
    publisherTopic = argv[2];
  subscriberTopic = argv[1];
  
  nodeHandle.reset(new ros::NodeHandle("~"));
 
  bool unreliable = false;
  nodeHandle->getParam("unreliable", unreliable);
  nodeHandle->getParam("lazy", lazy);

  if (unreliable)
    subscriberTransportHints.unreliable().reliable();

  subscribe();
  ros::spin();
  
  return 0;
}
