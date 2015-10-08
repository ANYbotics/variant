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

/** \file ArrayTypeTraits.h
  * \brief Header file providing the ArrayTypeTraits class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_ARRAY_TYPE_TRAITS_H
#define VARIANT_TOPIC_TOOLS_ARRAY_TYPE_TRAITS_H

#include <vector>

#include <boost/array.hpp>

namespace variant_topic_tools {
  /** \brief Array type traits
    */
  struct ArrayTypeTraits {
    template <class A> struct IsArray :
      public boost::false_type {
    };
    
    template <typename T> struct IsArray<std::vector<T> > :
      public boost::true_type {
    };
    
    template <typename T, size_t N> struct IsArray<boost::array<T, N> > :
      public boost::true_type {
    };
    
    template <class A> struct FromArray;
    
    template <typename T> struct FromArray<std::vector<T> > {
      typedef T ElementType;
      static const size_t NumElements = 0;
    };
    
    template <typename T, size_t N> struct FromArray<boost::array<T, N> > {
      typedef T ElementType;
      static const size_t NumElements = N;
    };
    
    template <typename T, size_t N> struct ToArray {
      typedef boost::array<T, N> ArrayType;
    };
    
    template <typename T> struct ToArray<T, 0> {
      typedef std::vector<T> ArrayType;
    };
  };
};

#endif
