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

/** \file Exceptions.h
  * \brief Header file defining exceptions for the variant topic tools
  */

#ifndef VARIANT_TOPIC_TOOLS_EXCEPTIONS_H
#define VARIANT_TOPIC_TOOLS_EXCEPTIONS_H

#include <ros/ros.h>

namespace variant_topic_tools {
  /** \brief Variant topic tools exception type
    */
  
  /** \brief Exception thrown in case of an invalid message data type
    */ 
  class InvalidDataTypeException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    InvalidDataTypeException(const std::string& dataType);
  };
  
  /** \brief Exception thrown in case of a message data type mismatch
    */ 
  class DataTypeMismatchException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    DataTypeMismatchException(const std::string& expectedDataType, const
      std::string& providedDataType);
  };
  
  /** \brief Exception thrown in case of a message MD5 sum mismatch
    */ 
  class MD5SumMismatchException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    MD5SumMismatchException(const std::string& expectedMD5Sum, const
      std::string& providedMD5Sum);
  };
  
  /** \brief Exception thrown in case of a bad message field name
    */ 
  class BadFieldNameException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    BadFieldNameException(const std::string& name);
  };
  
  /** \brief Exception thrown in case of a bad message field index
    */ 
  class BadFieldIndexException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    BadFieldIndexException(size_t index);
  };
  
  /** \brief Exception thrown in case of an error to parse the message
    *   definition
    */ 
  class DefinitionParseException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    DefinitionParseException(const std::string& dataType, const std::string&
      line, const std::string& what);
  };
  
  /** \brief Exception thrown in case of a package not found
    */ 
  class PackageNotFoundException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    PackageNotFoundException(const std::string& package);
  };
  
  /** \brief Exception thrown in case of an error to open a file
    */ 
  class FileOpenException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    FileOpenException(const std::string& filename);
  };
};

#endif
