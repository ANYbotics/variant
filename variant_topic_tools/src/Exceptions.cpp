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

InvalidDataTypeException::InvalidDataTypeException(const std::string&
    dataType) :
  ros::Exception("Data type ["+dataType+"] is invalid") {
}

DataTypeMismatchException::DataTypeMismatchException(const std::string&
    expectedDataType, const std::string& providedDataType) :
  ros::Exception("Provided data type ["+providedDataType+
    "] mismatches expected data type ["+expectedDataType+"]") {
}

MD5SumMismatchException::MD5SumMismatchException(const std::string&
    expectedMD5Sum, const std::string& providedMD5Sum) :
  ros::Exception("Provided MD5 sum ["+providedMD5Sum+
    "] mismatches expected MD5 sum ["+expectedMD5Sum+"]") {
}

BadFieldNameException::BadFieldNameException(const std::string& name) :
  ros::Exception("Field with name ["+name+"] does not exist") {
}

BadFieldIndexException::BadFieldIndexException(size_t index) :
  ros::Exception("Field index ["+boost::lexical_cast<std::string>(index)+
    "] out of range") {
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

}
