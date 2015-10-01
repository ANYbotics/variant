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

/** \file VariantCollection.h
  * \brief Header file providing the VariantCollection class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_VARIANT_COLLECTION_H
#define VARIANT_TOPIC_TOOLS_VARIANT_COLLECTION_H

#include <variant_topic_tools/SharedVariant.h>
#include <variant_topic_tools/Variant.h>

namespace variant_topic_tools {
  /** \brief Variant collection type
    */  
  class VariantCollection :
    public Variant {
  friend class Variant;
  public:
    /** \brief Default constructor
      */ 
    VariantCollection();
    
    /** \brief Copy constructor
      */ 
    VariantCollection(const VariantCollection& src);
    
    /** \brief Copy constructor (overloaded version taking a variant)
      */ 
    VariantCollection(const Variant& src);
    
    /** \brief Destructor
      */ 
    ~VariantCollection();
        
    /** \brief Retrieve the number of members of the variant collection
      */
    size_t getNumMembers() const;
    
    /** \brief Set a member of the variant collection by index
      */
    void setMember(size_t index, const Variant& member);
    
    /** \brief Set a member of the variant collection by name
      */
    void setMember(const std::string& name, const Variant& member);
    
    /** \brief Retrieve a member of the variant collection by index
      */
    SharedVariant getMember(size_t index) const;
    
    /** \brief Retrieve a member of the variant collection by name
      */
    SharedVariant getMember(const std::string& name) const;
    
    /** \brief Set a member value of the variant collection by index
      */    
    template <typename T> void setValue(size_t index, const T& value);
    
    /** \brief Set a member value of the variant collection by name
      */    
    template <typename T> void setValue(const std::string& name, const
      T& value);
    
    using Variant::setValue;
    
    /** \brief Retrieve a member value of the variant collection by index
      *   (non-const version)
      */
    template <typename T> T& getValue(size_t index);
    
    /** \brief Retrieve a member value of the variant collection by index
      *   (const version)
      */
    template <typename T> const T& getValue(size_t index) const;
    
    /** \brief Retrieve a member value of the variant collection by name
      *   (non-const version)
      */
    template <typename T> T& getValue(const std::string& name);
    
    /** \brief Retrieve a member value of the variant collection by name
      *   (const version)
      */
    template <typename T> const T& getValue(const std::string& name) const;
    
    using Variant::getValue;
    
    /** \brief True, if the variant collection contains the member with the
      *   specified name
      */
    bool hasMember(const std::string& name) const;

    /** \brief True, if the variant collection is empty
      */
    bool isEmpty() const;
    
    /** \brief Operator for retrieving the members of the variant collection
      *   by index
      */
    SharedVariant operator[](size_t index) const;
    
    /** \brief Operator for retrieving the members of the variant collection
      *   by name
      */
    SharedVariant operator[](const std::string& name) const;
    
  protected:
    /** \brief Variant collection value (abstract base)
      */
    class Value :
      public virtual Variant::Value {
    public:
      /** \brief Default constructor
        */ 
      Value();
      
      /** \brief Destructor
        */ 
      virtual ~Value();
              
      /** \brief Retrieve the number of members of the variant collection
        *   (abstract declaration)
        */
      virtual size_t getNumMembers() const = 0;
      
      /** \brief Set a member of the variant collection by index (abstract
        *   declaration)
        */
      virtual void setMember(size_t index, const Variant& member) = 0;
      
      /** \brief Set a member of the variant collection by name (abstract
        *   declaration)
        */
      virtual void setMember(const std::string& name, const Variant&
        member) = 0;
    
      /** \brief Recursively set a member of the variant collection by name
        */
      void setMember(const std::string& name, const Variant& member,
        size_t pos);
    
      /** \brief Retrieve a member of the variant collection by index
        *   (abstract declaration)
        */
      virtual SharedVariant getMember(size_t index) const = 0;
      
      /** \brief Retrieve a member of the variant collection by name
        *   (abstract declaration)
        */
      virtual SharedVariant getMember(const std::string& name) const = 0;
      
      /** \brief Recursively retrieve a member of the variant collection
        *   by name
        */
      SharedVariant getMember(const std::string& name, size_t pos) const;
    
      /** \brief True, if the variant collection contains the member with the
        *   specified name (abstract declaration)
        */
      virtual bool hasMember(const std::string& name) const = 0;
      
      /** \brief True, if the variant collection or any of its members
        *   contains the member with the specified name
        */
      bool hasMember(const std::string& name, size_t pos) const;
    
      /** \brief True, if this variant value equals another variant value
        *   (implementation)
        */
      bool isEqual(const Variant::Value& value) const;
      
      /** \brief Read the variant from a stream (implementation)
        */
      void read(std::istream& stream);
    
      /** \brief Write the variant collection member with the specified
        *   index to a stream (abstract declaration)
        */
      virtual void writeMember(std::ostream& stream, size_t index) const = 0;
      
      /** \brief Write this variant value to a stream (implementation)
        */
      void write(std::ostream& stream) const;      
    };
    
    /** \brief Constructor (overloaded version taking a data type)
      */ 
    VariantCollection(const DataType& type);
  };
};

#include <variant_topic_tools/VariantCollection.tpp>

#endif
