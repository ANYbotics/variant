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

template <typename T, size_t N>
ArrayDataType::ImplT<T, N>::ImplT() :
  Impl(typeid(T)) {
}

template <typename T, size_t N>
ArrayDataType::ImplT<T, N>::~ImplT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T, size_t N>
size_t ArrayDataType::ImplT<T, N>::getNumElements() const {
  return N;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class A> ArrayDataType ArrayDataType::create() {
  return ArrayDataType::template create<typename
    TypeTraits::FromArray<A>::ElementType,
    TypeTraits::FromArray<A>::NumElements>();
}

template <typename T, size_t N> ArrayDataType ArrayDataType::create() {
  ArrayDataType arrayDataType;
  arrayDataType.impl.reset(new ImplT<T, N>());
  
  return arrayDataType;
}

}
