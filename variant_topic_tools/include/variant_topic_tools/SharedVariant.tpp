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
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T> SharedVariant::SharedVariant(const T& src) {
  TypeTraits::assign(*this, src);
}


/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> void SharedVariant::TypeTraits::assign(SharedVariant&
    dst, const T& src, typename boost::enable_if<boost::is_base_of<Variant,
    T> >::type* dummy) {
  dst.type = src.type;
  dst.value = src.value;
}

template <typename T> void SharedVariant::TypeTraits::assign(SharedVariant&
    dst, const T& src, typename boost::disable_if<boost::is_base_of<Variant,
    T> >::type* dummy) {
  dst.template setValue<T>(src);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

template <typename T> SharedVariant& SharedVariant::operator=(const T& src) {
  TypeTraits::assign(*this, src);
  return *this;
}

}
