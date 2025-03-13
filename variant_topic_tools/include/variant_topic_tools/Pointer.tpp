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

#include <variant_topic_tools/Exceptions.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
Pointer<T>::Pointer(T* pointer) {
  if (pointer) this->impl.reset(new Impl(boost::shared_ptr<T>(pointer)));
}

template <typename T>
Pointer<T>::Pointer(const Pointer<T>& src) : impl(src.impl) {}

template <typename T>
Pointer<T>::~Pointer() {}

template <typename T>
Pointer<T>::ImplA::ImplA() {}

template <typename T>
Pointer<T>::ImplA::~ImplA() {}

template <typename T>
Pointer<T>::Impl::Impl(const boost::shared_ptr<T>& pointer) : pointer(pointer) {}

template <typename T>
Pointer<T>::Impl::~Impl() {}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
T* Pointer<T>::get() const {
  if (this->impl)
    return this->impl->get();
  else
    return 0;
}

template <typename T>
T* Pointer<T>::Impl::get() const {
  return this->pointer.get();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T>
void Pointer<T>::reset() {
  this->impl.reset();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

template <typename T>
T& Pointer<T>::operator*() const {
  T* pointer = this->operator->();

  if (pointer)
    return *pointer;
  else
    throw NullPointerException();
}

template <typename T>
T* Pointer<T>::operator->() const {
  if (this->impl)
    return this->impl->get();
  else
    return 0;
}

template <typename T>
Pointer<T>::operator void*() const {
  return this->operator->();
}

template <typename T>
bool Pointer<T>::operator==(const Pointer<T>& pointer) const {
  return (this->operator->() == pointer.operator->());
}

template <typename T>
bool Pointer<T>::operator!=(const Pointer<T>& pointer) const {
  return (this->operator->() != pointer.operator->());
}

}  // namespace variant_topic_tools
