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

/** \file TypeInfoHash.h
 * \brief Header file providing the TypeInfoHash class interface
 */

#ifndef VARIANT_TOPIC_TOOLS_TYPE_INFO_HASH_H
#define VARIANT_TOPIC_TOOLS_TYPE_INFO_HASH_H

#include <typeinfo>

namespace variant_topic_tools {
/** \brief Type information hash
 */
class TypeInfoHash {
 public:
  /** \brief Type information hash operator
   */
  inline std::size_t operator()(const std::type_info* typeInfo) const { return reinterpret_cast<std::size_t>(typeInfo); };
};
}  // namespace variant_topic_tools

#endif
