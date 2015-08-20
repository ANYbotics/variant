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

/** \file MessageField.h
  * \brief Header file providing the MessageField class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_FIELD_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_FIELD_H

#include <ros/ros.h>

#include <variant_topic_tools/MessageFieldCollection.h>
#include <variant_topic_tools/MessageType.h>

namespace variant_topic_tools {
  /** \brief Message field template
    */
  template <typename T> class MessageField :
    public MessageFieldCollection<T> {
  public:
    /** \brief Default constructor
      */ 
    MessageField(const std::string& name = std::string(), const
      T& value = T());
    
    /** \brief Copy constructor
      */ 
    MessageField(const MessageField<T>& src);
    
    /** \brief Destructor
      */ 
    virtual ~MessageField();
    
    /** \brief Set the name of the message field
      */
    void setName(const std::string& name);
    
    /** \brief Retrieve the name of the message field
      */
    const std::string& getName() const;
    
    /** \brief Set the value of the message field
      */
    void setValue(const T& value);
    
    /** \brief Retrieve the value of the message field (non-const version)
      */
    T& getValue();
    
    /** \brief Retrieve the value of the message field (const version)
      */
    const T& getValue() const;
    
    /** \brief True, if this message field is valid
      */ 
    bool isValid() const;
    
    /** \brief Clear the message field
      */
    void clear();
    
    /** \brief Write the message field to a stream
      */
    void write(std::ostream& stream, const std::string& indent =
      std::string()) const;
    
    /** \brief True, if this message field is equal to another message field
      */
    bool operator==(const MessageField<T>& field) const;
    
    /** \brief True, if this message field is not equal to another message
      *   field
      */
    bool operator!=(const MessageField<T>& field) const;
    
  protected:
    /** \brief Type traits
      */
    struct TypeTraits {
      template <typename U> struct HasIsValid {
        template <typename V, bool (V::*)() const> struct Test {};
        
        template <typename V> static char test(Test<V, &V::isValid>*);
        template <typename V> static int test(...);
        
        static const bool value = sizeof(test<U>(0)) == sizeof(char);
      };

      template <typename U> static bool isValid(const U& value,
        typename boost::enable_if_c<HasIsValid<U>::value>::type* = 0);
      template <typename U> static bool isValid(const U& value,
        typename boost::disable_if_c<HasIsValid<U>::value>::type* = 0);
    };

    /** \brief The name of this message field
      */ 
    std::string name;
    
    /** \brief The value of this message field
      */ 
    T value;
  };
  
  /** \brief Operator for writing the message field to a stream
    */
  template <typename T> std::ostream& operator<<(std::ostream& stream,
    const MessageField<T>& messageField);
};

#include <variant_topic_tools/MessageField.tpp>

#endif
