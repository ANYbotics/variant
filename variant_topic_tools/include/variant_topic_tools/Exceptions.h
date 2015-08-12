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
  /** \brief Exception thrown in case of an invalid operation
    */ 
  class InvalidOperationException :
    public ros::Exception {
  public:
    /** \brief Default constructor
      */
    InvalidOperationException();
  };
  
  /** \brief Exception thrown in case of an invalid data type
    */ 
  class InvalidDataTypeException :
    public ros::Exception {
  public:
    /** \brief Default constructor
      */
    InvalidDataTypeException();
  };
  
  /** \brief Exception thrown in case of an attempted modification of
    *   an immutable data type
    */ 
  class ImmutableDataTypeException :
    public ros::Exception {
  public:
    /** \brief Default constructor
      */
    ImmutableDataTypeException();
  };
  
  /** \brief Exception thrown in case of a non-existent data type
    */ 
  class NoSuchDataTypeException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    NoSuchDataTypeException(const std::string& identifier);
  };
  
  /** \brief Exception thrown in case of an ambiguous data type identifier
    */ 
  class AmbiguousDataTypeIdentifierException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    AmbiguousDataTypeIdentifierException(const std::string& identifier);
  };
  
  /** \brief Exception thrown in case of a data type mismatch
    */ 
  class DataTypeMismatchException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    DataTypeMismatchException(const std::string& expectedDataType, const
      std::string& providedDataType);
  };
  
  /** \brief Exception thrown in case of an invalid message member
    */ 
  class InvalidMessageMemberException :
    public ros::Exception {
  public:
    /** \brief Default constructor
      */
    InvalidMessageMemberException();
  };
  
  /** \brief Exception thrown in case of a non-existent message member
    */ 
  class NoSuchMessageMemberException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    NoSuchMessageMemberException(size_t index);
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
  
  /** \brief Exception thrown in case of an ambiguous message field name
    */ 
  class AmbiguousMessageFieldNameException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    AmbiguousMessageFieldNameException(const std::string& name);
  };
  
  /** \brief Exception thrown in case of a non-existent message field
    */ 
  class NoSuchMessageFieldException :
    public ros::Exception {
  public:
    /** \brief Constructor (overloaded version taking a field index)
      */
    NoSuchMessageFieldException(size_t index);
    
    /** \brief Constructor (overloaded version taking a field name)
      */
    NoSuchMessageFieldException(const std::string& name);
  };
  
  /** \brief Exception thrown in case of an invalid message type
    */ 
  class InvalidMessageTypeException :
    public ros::Exception {
  public:
    /** \brief Constructor
      */
    InvalidMessageTypeException(const std::string& invalidMessageType);
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
  
  /** \brief Exception thrown in case of a package not being found
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
