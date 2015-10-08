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

/** \file MessageFieldTypeTraits.h
  * \brief Header file providing the MessageFieldTypeTraits class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MESSAGE_FIELD_TYPE_TRAITS_H
#define VARIANT_TOPIC_TOOLS_MESSAGE_FIELD_TYPE_TRAITS_H

namespace variant_topic_tools {
  /** \brief Message field type traits
    */
  struct MessageFieldTypeTraits {
    template <typename U> struct HasIsValid {
      template <typename V, bool (V::*)() const> struct Test {};
      
      template <typename V> static char test(Test<V, &V::isValid>*);
      template <typename V> static int test(...);
      
      static const bool value = sizeof(test<U>(0)) == sizeof(char);
    };
  };
};

#endif
