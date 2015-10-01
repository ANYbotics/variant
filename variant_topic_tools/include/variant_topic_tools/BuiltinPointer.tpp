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
BuiltinPointer<T>::BuiltinPointer(T* builtin) {
  this->impl.reset(new Impl(BuiltinPtr(builtin)));
}

template <typename T>
BuiltinPointer<T>::BuiltinPointer(const BuiltinPtr& builtin) {
  this->impl.reset(new Impl(builtin));
}

template <typename T>
BuiltinPointer<T>::BuiltinPointer(const BuiltinPointer<T>& src) :
  Pointer<T>(src) {
}

template <typename T>
BuiltinPointer<T>::BuiltinPointer(const Pointer<T>& src) :
  Pointer<T>(src) {
  if (this->impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(this->impl));
}

template <typename T>
BuiltinPointer<T>::~BuiltinPointer() {
}

template <typename T>
BuiltinPointer<T>::Impl::Impl(const BuiltinPtr& builtin) :
  builtin(builtin) {
}

template <typename T>
BuiltinPointer<T>::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
T* BuiltinPointer<T>::Impl::get() const {
  return this->builtin.get();
}

}
