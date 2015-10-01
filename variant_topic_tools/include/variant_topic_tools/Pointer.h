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

/** \file Pointer.h
  * \brief Header file providing the Pointer class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_POINTER_H
#define VARIANT_TOPIC_TOOLS_POINTER_H

#include <ros/ros.h>

#include <variant_topic_tools/Forwards.h>

namespace variant_topic_tools {
  /** \brief Shared pointer
    */
  template <typename T> class Pointer {
  public:
    /** \brief Default constructor
      */ 
    Pointer();
    
    /** \brief Copy constructor
      */ 
    Pointer(const Pointer<T>& src);
    
    /** \brief Destructor
      */ 
    ~Pointer();

    /** \brief Retrieve the stored pointer
      */
    T* get() const;
      
    /** \brief Reference operator
      */
    T& operator*() const;
    
    /** \brief Pointer operator
      */
    T* operator->() const;
    
    /** \brief Void pointer conversion
      */
    operator void*() const;
    
    /** \brief Equality comparison operator
      */
    bool operator==(const Pointer<T>& pointer) const;
    
    /** \brief Inequality comparison operator
      */
    bool operator!=(const Pointer<T>& pointer) const;
    
  protected:
    /** \brief Pointer implementation (abstract base)
      */
    class Impl {
    public:
      /** \brief Constructor
        */
      Impl();
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the stored pointer (abstract declaration)
        */
      virtual T* get() const = 0;
    };
    
    /** \brief Declaration of the pointer implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the pointer implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The pointer's implementation
      */
    ImplPtr impl;
  };
};

#include <variant_topic_tools/Pointer.tpp>

#endif
