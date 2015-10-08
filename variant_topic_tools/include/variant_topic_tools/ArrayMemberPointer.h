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

/** \file ArrayMemberPointer.h
  * \brief Header file providing the ArrayMemberPointer class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_ARRAY_MEMBER_POINTER_H
#define VARIANT_TOPIC_TOOLS_ARRAY_MEMBER_POINTER_H

#include <variant_topic_tools/Pointer.h>

namespace variant_topic_tools {
  /** \brief Shared pointer to an array member
    */
  template <class A, typename T> class ArrayMemberPointer :
    public Pointer<T> {
  public:
    /** \brief Definition of the array pointer type
      */
    typedef Pointer<A> ArrayPtr;
      
    /** \brief Default constructor
      */ 
    ArrayMemberPointer(A* array = 0, size_t index = 0);
    
    /** \brief Constructor (overloaded version taking an array pointer
      *   and an index)
      */ 
    ArrayMemberPointer(const ArrayPtr& array, size_t index);
    
    /** \brief Copy constructor
      */ 
    ArrayMemberPointer(const ArrayMemberPointer<A, T>& src);
    
    /** \brief Copy constructor (overloaded version taking a pointer)
      */ 
    ArrayMemberPointer(const Pointer<T>& src);
    
    /** \brief Destructor
      */ 
    ~ArrayMemberPointer();

    /** \brief Set the array
      */
    void setArray(const ArrayPtr& array);
    
    /** \brief Retrieve the array
      */
    const ArrayPtr& getArray() const;
    
    /** \brief Set the array member index
      */
    void setIndex(size_t index);
    
    /** \brief Retrieve the array member index
      */
    size_t getIndex() const;
    
  protected:    
    /** \brief Array member pointer implementation
      */
    class Impl :
      public Pointer<T>::Impl {
    public:
      /** \brief Default constructor
        */
      Impl(const ArrayPtr& array = ArrayPtr(), size_t index = 0);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the stored pointer (implementation)
        */
      T* get() const;
      
      /** \brief The shared array
        */ 
      ArrayPtr array;
      
      /** \brief The member index
        */ 
      size_t index;
    };
  };
};

#include <variant_topic_tools/ArrayMemberPointer.tpp>

#endif
