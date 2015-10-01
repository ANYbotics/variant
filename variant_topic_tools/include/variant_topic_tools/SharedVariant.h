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

/** \file SharedVariant.h
  * \brief Header file providing the SharedVariant class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_SHARED_VARIANT_H
#define VARIANT_TOPIC_TOOLS_SHARED_VARIANT_H

#include <variant_topic_tools/Variant.h>

namespace variant_topic_tools {
  /** \brief Shared variant type
    */  
  class SharedVariant :
    public Variant {
  public:
    /** \brief Default constructor
      */ 
    SharedVariant();
    
    /** \brief Copy constructor
      */ 
    template <typename T> SharedVariant(const T& src);
    
    /** \brief Destructor
      */ 
    ~SharedVariant();
    
    /** \brief Assignment operator
      */
    template <typename T> SharedVariant& operator=(const T& src);
    
  protected:
    /** \brief Type traits
      */
    struct TypeTraits {
      template <typename T> static void assign(SharedVariant& dst, const T&
        src, typename boost::enable_if<boost::is_base_of<Variant, T> >::type*
        dummy = 0);
      template <typename T> static void assign(SharedVariant& dst, const T&
        src, typename boost::disable_if<boost::is_base_of<Variant, T> >::type*
        dummy = 0);
    };
  };
};

#include <variant_topic_tools/SharedVariant.tpp>

#endif
