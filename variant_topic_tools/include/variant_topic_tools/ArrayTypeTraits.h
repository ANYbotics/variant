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
  * \brief Header file providing the array type traits
  */

#ifndef VARIANT_TOPIC_TOOLS_ARRAY_TYPE_TRAITS_H
#define VARIANT_TOPIC_TOOLS_ARRAY_TYPE_TRAITS_H

#include <vector>

#include <boost/array.hpp>
#include <boost/type_traits.hpp>

#include <variant_topic_tools/BuiltinTypeTraits.h>
#include <variant_topic_tools/MessageTypeTraits.h>

namespace variant_topic_tools {
  namespace type_traits {
    template <typename T> struct IsArray :
      public boost::false_type {
    };
    
    template <typename T> struct IsArray<T[]> :
      public boost::true_type {
    };
    
    template <typename T, size_t N> struct IsArray<T[N]> :
      public boost::true_type {
    };
    
    template <size_t N> struct IsArray<char[N]> :
      public boost::false_type {
    };
    
    template <typename T, typename D = void> struct ArrayMemberType {
      typedef T ValueType;
    };
    
    template <typename D> struct ArrayMemberType<bool, D> {
      typedef uint8_t ValueType;
    };
    
    template <typename T> struct ArrayType;
    
    template <typename T> struct ArrayType<T[]> {
      typedef T MemberType;
      typedef typename ArrayMemberType<T>::ValueType MemberValueType;
      typedef std::vector<MemberValueType> ValueType;
      static const size_t NumMembers = 0;
      typedef boost::false_type IsFixedSize;
    };
    
    template <typename T, size_t N> struct ArrayType<T[N]> {
      typedef T MemberType;
      typedef typename ArrayMemberType<T>::ValueType MemberValueType;
      typedef boost::array<MemberValueType, N> ValueType;
      static const size_t NumMembers = N;
      typedef boost::true_type IsFixedSize;
    };
    
    template <typename T> struct ToArrayType {
      typedef T ArrayType;
    };
    
    template <typename T> struct ToArrayType<std::vector<T> > {
      typedef T ArrayType[];
    };
    
    template <typename T, size_t N> struct ToArrayType<boost::array<T, N> > {
      typedef T ArrayType[N];
    };
  };
};

#endif
