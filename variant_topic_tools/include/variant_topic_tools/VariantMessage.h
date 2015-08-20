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

/** \file VariantMessage.h
  * \brief Header file providing the VariantMessage class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_VARIANT_MESSAGE_H
#define VARIANT_TOPIC_TOOLS_VARIANT_MESSAGE_H

#include <variant_topic_tools/MessageDataType.h>
#include <variant_topic_tools/MessageFieldCollection.h>
#include <variant_topic_tools/VariantCollection.h>

namespace variant_topic_tools {
  /** \brief Variant message type
    */  
  class VariantMessage :
    public VariantCollection {
  friend class MessageDataType;
  friend class Variant;
  public:
    /** \brief Default constructor
      */ 
    VariantMessage();
    
    /** \brief Copy constructor
      */ 
    VariantMessage(const VariantMessage& src);
    
    /** \brief Copy constructor (overloaded version taking a variant)
      */ 
    VariantMessage(const Variant& src);
    
    /** \brief Destructor
      */ 
    ~VariantMessage();
        
  protected:
    /** \brief Variant message value
      */
    class Value :
      public VariantCollection::Value {
    public:
      /** \brief Default constructor
        */ 
      Value(const MessageFieldCollection<Variant>& members =
        MessageFieldCollection<Variant>());
      
      /** \brief Copy constructor
        */ 
      Value(const Value& src);
      
      /** \brief Destructor
        */ 
      virtual ~Value();
      
      /** \brief Retrieve the number of members of the variant collection
        *   (implementation)
        */
      size_t getNumMembers() const;
      
      /** \brief Retrieve a member of the variant collection by index
        *   (implementation of the non-const version)
        */
      Variant& getMember(size_t index);
      
      /** \brief Retrieve a member of the variant collection by index
        *   (implementation of the const version)
        */
      const Variant& getMember(size_t index) const;
      
      /** \brief Retrieve a member of the variant collection by name
        *   (implementation of the non-const version)
        */
      Variant& getMember(const std::string& name);
      
      /** \brief Retrieve a member of the variant collection by name
        *   (implementation of the const version)
        */
      const Variant& getMember(const std::string& name) const;
      
      /** \brief True, if the variant collection contains the member with the
        *   specified name (implementation)
        */
      bool hasMember(const std::string& name) const;
      
      /** \brief Clone this variant value (implementation)
        */
      ValuePtr clone() const;
      
      /** \brief Write the variant collection member with the specified
        *   index to a stream (implementation)
        */
      void writeMember(std::ostream& stream, size_t index) const;
      
      /** \brief The message members
        */
      MessageFieldCollection<Variant> members;
    };
    
    /** \brief Constructor (overloaded version taking a message data type
      *   and a message field collection)
      */ 
    VariantMessage(const MessageDataType& type, const
      MessageFieldCollection<Variant>& members);
  };
};

#endif
