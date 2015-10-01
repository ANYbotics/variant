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

#include <boost/lexical_cast.hpp>

#include "variant_topic_tools/Exceptions.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

NullPointerException::NullPointerException() :
  ros::Exception("Attempted null pointer operation") {
}

InvalidOperationException::InvalidOperationException(const std::string& what) :
  ros::Exception("Attempted execution of an invalid operation: "+what) {
}

InvalidDataTypeException::InvalidDataTypeException() :
  ros::Exception("Attempted use of an invalid data type") {
}

ImmutableDataTypeException::ImmutableDataTypeException() :
  ros::Exception("Attempted modification of an immutable data type") {
}

NoSuchDataTypeException::NoSuchDataTypeException(const std::string&
    identifier) :
  ros::Exception("Data type ["+identifier+"] does not exist") {
}

AmbiguousDataTypeIdentifierException::AmbiguousDataTypeIdentifierException(
    const std::string& identifier) :
  ros::Exception("Data type identifier ["+identifier+
    "] is used ambiguously") {
}

DataTypeMismatchException::DataTypeMismatchException(const std::string&
    expectedDataType, const std::string& providedDataType) :
  ros::Exception("Provided data type ["+providedDataType+
    "] mismatches expected data type ["+expectedDataType+"]") {
}

AmbiguousMemberNameException::AmbiguousMemberNameException(
    const std::string& name) :
  ros::Exception("Member name ["+name+"] is used ambiguously") {
}

NoSuchMemberException::NoSuchMemberException(size_t index) :
  ros::Exception("Member with index ["+boost::lexical_cast<std::string>(index)+
    "] does not exist") {
}

NoSuchMemberException::NoSuchMemberException(const std::string& name) :
  ros::Exception("Member with name ["+name+"] does not exist") {
}

InvalidMessageMemberException::InvalidMessageMemberException() :
  ros::Exception("Attempted use of an invalid message member") {
}

MD5SumMismatchException::MD5SumMismatchException(const std::string&
    expectedMD5Sum, const std::string& providedMD5Sum) :
  ros::Exception("Provided MD5 sum ["+providedMD5Sum+
    "] mismatches expected MD5 sum ["+expectedMD5Sum+"]") {
}

InvalidMessageTypeException::InvalidMessageTypeException(const std::string&
    invalidMessageType) :
  ros::Exception("Message type ["+invalidMessageType+"] is invalid") {
}
  
DefinitionParseException::DefinitionParseException(const std::string&
    dataType, const std::string& line, const std::string& what) :
  ros::Exception("Error parsing the definition for ["+dataType+"]: "+
    what+"\n"+line) {
}

PackageNotFoundException::PackageNotFoundException(const std::string&
    package) :
  ros::Exception("Package ["+package+"] not found") {
}

FileOpenException::FileOpenException(const std::string& filename) :
  ros::Exception("Error opening file ["+filename+"]") {
}

InvalidSerializerException::InvalidSerializerException() :
  ros::Exception("Attempted use of an invalid serializer") {
}

}
