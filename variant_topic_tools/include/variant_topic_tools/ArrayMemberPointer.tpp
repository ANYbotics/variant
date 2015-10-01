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

template <class A, typename T>
ArrayMemberPointer<A, T>::ArrayMemberPointer(A* array, size_t index) {
  this->impl.reset(new Impl(ArrayPtr(array), index));
}

template <class A, typename T>
ArrayMemberPointer<A, T>::ArrayMemberPointer(const ArrayPtr& array,
    size_t index) {
  this->impl.reset(new Impl(array, index));
}

template <class A, typename T>
ArrayMemberPointer<A, T>::ArrayMemberPointer(const ArrayMemberPointer<A, T>&
    src) :
  Pointer<T>(src) {
}

template <class A, typename T>
ArrayMemberPointer<A, T>::ArrayMemberPointer(const Pointer<T>& src) :
  Pointer<T>(src) {
  if (this->impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(this->impl));
}

template <class A, typename T>
ArrayMemberPointer<A, T>::~ArrayMemberPointer() {
}

template <class A, typename T>
ArrayMemberPointer<A, T>::Impl::Impl(const ArrayPtr& array, size_t index) :
  array(array),
  index(index) {
}

template <class A, typename T>
ArrayMemberPointer<A, T>::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <class A, typename T>
void ArrayMemberPointer<A, T>::setArray(const ArrayPtr& array) {
  boost::static_pointer_cast<Impl>(this->impl)->array = array;
}

template <class A, typename T>
const typename ArrayMemberPointer<A, T>::ArrayPtr&
    ArrayMemberPointer<A, T>::getArray() const {
  return boost::static_pointer_cast<Impl>(this->impl)->array;
}
    
template <class A, typename T>
void ArrayMemberPointer<A, T>::setIndex(size_t index) {
  boost::static_pointer_cast<Impl>(this->impl)->index = index;
}

template <class A, typename T>
size_t ArrayMemberPointer<A, T>::getIndex() const {
  return boost::static_pointer_cast<Impl>(this->impl)->index;
}

template <class A, typename T>
T* ArrayMemberPointer<A, T>::Impl::get() const {
  if (this->array)
    return &((*this->array)[this->index]);
  else
    return 0;
}

}
