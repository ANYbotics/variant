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

#include <climits>

#include "variant_topic_tools/BuiltinVariant.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

BuiltinVariant::BuiltinVariant() = default;

BuiltinVariant::BuiltinVariant(const BuiltinVariant& src) : Variant(src) {}

BuiltinVariant::BuiltinVariant(const Variant& src) : Variant(src) {
  if (value) {
    BOOST_ASSERT(boost::dynamic_pointer_cast<Value>(value));
  }
}

BuiltinVariant::~BuiltinVariant() = default;

BuiltinVariant::Value::Value() = default;

BuiltinVariant::Value::~Value() = default;

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

double BuiltinVariant::getNumericValue() const {
  if (value) {
    return boost::dynamic_pointer_cast<Value>(value)->getNumericValue();
  } else {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace variant_topic_tools
