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

template <typename T>
ArrayMemberPointer<T>::ArrayMemberPointer(ValueType* array, size_t index) {
  this->impl.reset(new Impl(Pointer<ValueType>(array), index));
}

template <typename T>
ArrayMemberPointer<T>::ArrayMemberPointer(const Pointer<ValueType>& array,
    size_t index) {
  this->impl.reset(new Impl(array, index));
}

template <typename T>
ArrayMemberPointer<T>::ArrayMemberPointer(const ArrayMemberPointer<T>&
    src) :
  Pointer<MemberValueType>(src) {
}

template <typename T>
ArrayMemberPointer<T>::ArrayMemberPointer(const Pointer<MemberValueType>&
    src) :
  Pointer<MemberValueType>(src) {
  if (this->impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(this->impl));
}

template <typename T>
ArrayMemberPointer<T>::~ArrayMemberPointer() {
}

template <typename T>
ArrayMemberPointer<T>::Impl::Impl(const Pointer<ValueType>& array, size_t
    index) :
  array(array),
  index(index) {
}

template <typename T>
ArrayMemberPointer<T>::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
void ArrayMemberPointer<T>::setArray(const Pointer<ValueType>& array) {
  boost::static_pointer_cast<Impl>(this->impl)->array = array;
}

template <typename T>
const Pointer<typename ArrayMemberPointer<T>::ValueType>&
    ArrayMemberPointer<T>::getArray() const {
  return boost::static_pointer_cast<Impl>(this->impl)->array;
}
    
template <typename T>
void ArrayMemberPointer<T>::setIndex(size_t index) {
  boost::static_pointer_cast<Impl>(this->impl)->index = index;
}

template <typename T>
size_t ArrayMemberPointer<T>::getIndex() const {
  return boost::static_pointer_cast<Impl>(this->impl)->index;
}

template <typename T>
typename ArrayMemberPointer<T>::MemberValueType* ArrayMemberPointer<T>::
    Impl::get() const {
  if (this->array)
    return &((*this->array)[this->index]);
  else
    return 0;
}

}
