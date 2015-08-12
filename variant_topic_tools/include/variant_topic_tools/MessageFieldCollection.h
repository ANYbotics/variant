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

/** \file MessageFieldCollection.h
  * \brief Header file providing the MessageFieldCollection class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_FIELD_COLLECTION_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_FIELD_COLLECTION_H

#include <vector>

#include <boost/unordered_map.hpp>

#include <ros/ros.h>

#include <variant_topic_tools/Forwards.h>

namespace variant_topic_tools {
  /** \brief Templated message field collection
    */
  template <typename T> class MessageFieldCollection {
  public:
    /** \brief Default constructor
      */ 
    MessageFieldCollection();
    
    /** \brief Copy constructor
      */ 
    MessageFieldCollection(const MessageFieldCollection<T>& src);
    
    /** \brief Destructor
      */ 
    virtual ~MessageFieldCollection();
    
    /** \brief Access the number of fields of the message field collection
      */
    size_t getNumFields() const;
    
    /** \brief Access a field of the message field collection by index
      *   (non-const version)
      */
    MessageField<T>& getField(size_t index);
    
    /** \brief Access a field of the message field collection by index
      *   (const version)
      */
    const MessageField<T>& getField(size_t index) const;
    
    /** \brief Access a field of the message field collection by name
      *   (non-const version)
      */
    MessageField<T>& getField(const std::string& name);
    
    /** \brief Access a field of the message field collection by name
      *   (const version)
      */
    const MessageField<T>& getField(const std::string& name) const;
    
    /** \brief True, if the message field collection contains the
      *   field with the specified name
      */
    bool hasField(const std::string& name) const;

    /** \brief True, if the message field collections is empty
      */
    bool isEmpty() const;

    /** \brief Append a field to the message field collection
      */
    void appendField(const MessageField<T>& field);
    
    /** \brief Append a field to the message field collection (overloaded
      *   version taking a field name and a field value)
      */
    void appendField(const std::string& name, const T& value = T());
    
    /** \brief Merge this message field collection with another message
      *   field collection
      */
    void merge(const MessageFieldCollection<T>& collection);
    
    /** \brief Clear the message field collection
      */
    virtual void clear();
    
    /** \brief Operator for accessing the fields of the message field
      *   collection by index (non-const version)
      */
    MessageField<T>& operator[](size_t index);
    
    /** \brief Operator for accessing the fields of the message field
      *   collection by index (const version)
      */
    const MessageField<T>& operator[](size_t index) const;
    
    /** \brief Operator for accessing the fields of the message field
      *   collection by name (non-const version)
      */
    MessageField<T>& operator[](const std::string& name);
    
    /** \brief Operator for accessing the fields of the message field
      *   collection by name (const version)
      */
    const MessageField<T>& operator[](const std::string& name) const;
    
    /** \brief Operator for appending a field to the message field collection
      */
    MessageFieldCollection<T>& operator+=(const MessageField<T>& field);
    
  protected:
    /** \brief Declaration of the message field pointer type
      */
    typedef boost::shared_ptr<MessageField<T> > MessageFieldPtr;
    
    /** \brief Declaration of the message field weak pointer type
      */
    typedef boost::weak_ptr<MessageField<T> > MessageFieldWPtr;
    
    /** \brief The message fields of the collection in order
      */
    std::vector<MessageFieldPtr> fieldsInOrder;
    
    /** \brief The message fields of the collection by name
      */
    boost::unordered_map<std::string, MessageFieldPtr> fieldsByName;
    
    /** \brief Recursively retrieve a field of the message field collection
      *   by name
      */
    MessageField<T>& getField(const std::string& name, size_t pos) const;
    
    /** \brief True, if the message field collection or any of its fields
      *   contains the field with the specified name
      */
    bool hasField(const std::string& name, size_t pos) const;
  };
};

#include <variant_topic_tools/MessageFieldCollection.tpp>

#endif
