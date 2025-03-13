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

#include <algorithm>
#include <fstream>
#include <list>
#include <set>
#include <sstream>

#include <boost/unordered_map.hpp>
#include <utility>

#include <ros/package.h>

#include "variant_topic_tools/DataTypeRegistry.h"
#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/MessageDataType.h"
#include "variant_topic_tools/MessageDefinitionParser.h"
#include "variant_topic_tools/MessageType.h"
#include "variant_topic_tools/MessageTypeParser.h"
#include "variant_topic_tools/Publisher.h"
#include "variant_topic_tools/Subscriber.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

MessageType::MessageType(std::string dataType, std::string md5Sum, std::string definition)
    : dataType(std::move(dataType)), md5Sum(std::move(md5Sum)), definition(std::move(definition)) {}

MessageType::MessageType(const MessageDataType& dataType)
    : dataType(dataType.getIdentifier()), md5Sum(dataType.getMD5Sum()), definition(dataType.getDefinition()) {}

MessageType::MessageType(const MessageType& src) = default;

MessageType::~MessageType() = default;

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void MessageType::setDataType(const std::string& dataType) {
  this->dataType = dataType;
}

const std::string& MessageType::getDataType() const {
  return dataType;
}

void MessageType::setMD5Sum(const std::string& md5Sum) {
  this->md5Sum = md5Sum;
}

const std::string& MessageType::getMD5Sum() const {
  return md5Sum;
}

void MessageType::setDefinition(const std::string& definition) {
  this->definition = definition;
}

const std::string& MessageType::getDefinition() const {
  return definition;
}

bool MessageType::isValid() const {
  return !md5Sum.empty() && ((md5Sum == "*") || (md5Sum.length() == 32)) && !dataType.empty() && !definition.empty();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void MessageType::load(const std::string& messageDataType) {
  clear();

  std::string messagePackage;
  std::string messagePlainType;
  if (!MessageTypeParser::matchType(messageDataType, messagePackage, messagePlainType)) {
    throw InvalidMessageTypeException(messageDataType);
  }

  DataTypeRegistry registry;
  boost::unordered_map<std::string, std::string> definitions;
  std::list<std::string> typesInOrder;
  std::set<std::string> requiredTypes;

  requiredTypes.insert(messageDataType);
  typesInOrder.push_back(messageDataType);

  auto it = typesInOrder.begin();

  while (it != typesInOrder.end()) {
    std::string package;
    std::string plainType;

    if (!MessageTypeParser::matchType(*it, package, plainType)) {
      throw InvalidMessageTypeException(*it);
    }

    if (package.empty()) {
      if (plainType == "Header") {
        package = "std_msgs";
      } else {
        throw InvalidMessageTypeException(*it);
      }
    }

    std::string packagePath = ros::package::getPath(package);
    if (packagePath.empty()) {
      throw PackageNotFoundException(package);
    }

    std::string messageFilename(packagePath + "/msg/" + plainType + ".msg");
    std::ifstream messageFile(messageFilename.c_str());
    std::string messageDefinition;

    if (messageFile.is_open()) {
      messageFile.seekg(0, std::ios::end);
      messageDefinition.reserve(messageFile.tellg());
      messageFile.seekg(0, std::ios::beg);

      messageDefinition.assign((std::istreambuf_iterator<char>(messageFile)), std::istreambuf_iterator<char>());
    } else {
      throw FileOpenException(messageFilename);
    }

    messageFile.close();

    if (!messageDefinition.empty()) {
      std::istringstream stream(messageDefinition);
      std::string line;

      while (std::getline(stream, line)) {
        std::string memberName;
        std::string memberType;
        size_t memberSize = 0;

        if (MessageDefinitionParser::matchArray(line, memberName, memberType, memberSize) ||
            MessageDefinitionParser::match(line, memberName, memberType)) {
          std::string memberPackage;
          std::string plainMemberType;

          if (!MessageTypeParser::matchType(memberType, memberPackage, plainMemberType)) {
            throw InvalidMessageTypeException(memberType);
          }

          if (!registry.getDataType(memberType).isBuiltin()) {
            if (memberPackage.empty()) {
              if (plainMemberType == "Header") {
                memberPackage = "std_msgs";
              } else {
                memberPackage = package;
              }

              memberType = memberPackage + "/" + plainMemberType;
            }

            if (requiredTypes.find(memberType) == requiredTypes.end()) {
              requiredTypes.insert(memberType);
              typesInOrder.push_back(memberType);
            }
          }
        }
      }
    }

    definitions[*it] = messageDefinition;
    ++it;
  }

  for (auto& it : typesInOrder) {
    if (!definition.empty()) {
      definition += "\n" + std::string(80, '=') + "\n";
      definition += "MSG: " + it + "\n";
    }

    definition += definitions[it];
  }

  if (!definition.empty()) {
    dataType = messageDataType;
  }
}

void MessageType::clear() {
  dataType.clear();
  md5Sum = "*";
  definition.clear();
}

void MessageType::write(std::ostream& stream) const {
  stream << dataType;
}

Publisher MessageType::advertise(ros::NodeHandle& nodeHandle, const std::string& topic, size_t queueSize, bool latch,
                                 const ros::SubscriberStatusCallback& connectCallback) {
  Publisher publisher;

  if (isValid()) {
    publisher.impl.reset(new Publisher::Impl(nodeHandle, *this, topic, queueSize, latch, connectCallback));
  }

  return publisher;
}

Subscriber MessageType::subscribe(ros::NodeHandle& nodeHandle, const std::string& topic, size_t queueSize,
                                  const SubscriberCallback& callback) {
  Subscriber subscriber;

  subscriber.impl.reset(new Subscriber::Impl(nodeHandle, *this, topic, queueSize, callback));

  return subscriber;
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

bool MessageType::operator==(const MessageType& type) const {
  return (dataType == type.dataType) && (md5Sum == type.md5Sum);
}

bool MessageType::operator!=(const MessageType& type) const {
  return (dataType != type.dataType) || (md5Sum != type.md5Sum);
}

std::ostream& operator<<(std::ostream& stream, const MessageType& messageType) {
  messageType.write(stream);
  return stream;
}

}  // namespace variant_topic_tools
