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

#include <variant_msgs/Test.h>

#include <boost/variant.hpp>

ros::NodeHandlePtr nodeHandle;

ros::Publisher publisher;
std::string publisherTopic;
size_t publisherQueueSize = 100;
double publisherRate = 10.0;

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

void advertise() {
  publisher = nodeHandle->advertise<variant_msgs::Test>(publisherTopic,
    publisherQueueSize);
}

void callback(const ros::TimerEvent& timerEvent) {
  variant_msgs::Test message;
  
  message.builtin_string = "builtin_string";
  message.string.data = "string";
  
  publisher.publish(message);
}

template <typename T> class get_as : public boost::static_visitor<T> {
public:
  template <typename U> T operator()(U& operand) const {
    return operand;
  };
};

int main(int argc, char **argv) {
  if (argc < 2) {
    printf("\nusage: pub_test TEST_TOPIC [RATE]\n\n");
    return 1;
  }
  
  if (!getTopicBase((argv[1]), publisherTopic)) {
    ROS_ERROR("Failed to extract topic base from [%s]", argv[1]);
    return 1;
  }
  
  if (argc == 3)
    publisherRate = atof(argv[2]);

  ros::init(argc, argv, publisherTopic+"_pub_test",
    ros::init_options::AnonymousName);
  
  publisherTopic = argv[1];
  
  nodeHandle.reset(new ros::NodeHandle("~"));

  advertise();
  ros::Timer timer = nodeHandle->createTimer(
    ros::Rate(publisherRate).expectedCycleTime(), &callback);
  
  ros::spin();
  
  return 0;
}
