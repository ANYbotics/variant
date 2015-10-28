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

/** \file BuiltinTypeTraits.h
  * \brief Header file providing the built-in type traits
  */

#ifndef VARIANT_TOPIC_TOOLS_BUILTIN_TYPE_TRAITS_H
#define VARIANT_TOPIC_TOOLS_BUILTIN_TYPE_TRAITS_H

#include <string>

#include <boost/type_traits.hpp>

#include <ros/duration.h>
#include <ros/time.h>

namespace variant_topic_tools {
  namespace type_traits {
    template <typename T, typename D = void> struct IsBuiltin :
      public boost::type_traits::ice_or<
        boost::is_integral<T>::value,
        boost::is_floating_point<T>::value> {
    };
    
    template <typename D> struct IsBuiltin<std::string, D> :
      public boost::true_type {
    };
    
    template <typename D> struct IsBuiltin<ros::Duration, D> :
      public boost::true_type {
    };
    
    template <typename D> struct IsBuiltin<ros::Time, D> :
      public boost::true_type {
    };
    
    template <typename T, typename D = void> struct BuiltinType {
      typedef T ValueType;
      typedef typename boost::type_traits::ice_or<
        boost::is_integral<T>::value,
        boost::is_floating_point<T>::value> IsNumeric;
    };
    
    template <typename D> struct BuiltinType<bool, D> {
      typedef uint8_t ValueType;
      typedef boost::true_type IsNumeric;
    };
    
    template <typename T> struct ToBuiltinType {
      typedef T BuiltinType;
    };
    
    template <size_t N> struct ToBuiltinType<char[N]> {
      typedef std::string BuiltinType;
    };
  };
};

#endif
