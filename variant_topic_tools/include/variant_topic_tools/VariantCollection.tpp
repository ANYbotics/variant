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

namespace variant_topic_tools {

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T> void VariantCollection::setValue(size_t index, const T&
    value) {
  this->setMember(index, Variant(value));
}

template <typename T> void VariantCollection::setValue(const std::string&
    name, const T& value) {
  this->setMember(name, Variant(value));
}

template <typename T> T& VariantCollection::getValue(size_t index) {
  return this->getMember(index).template getValue<T>();
}

template <typename T> const T& VariantCollection::getValue(size_t index)
    const {
  return this->getMember(index).template getValue<T>();
}

template <typename T> T& VariantCollection::getValue(const std::string& name) {
  return this->getMember(name).template getValue<T>();
}

template <typename T> const T& VariantCollection::getValue(const std::string&
    name) const {
  return this->getMember(name).template getValue<T>();
}

}
