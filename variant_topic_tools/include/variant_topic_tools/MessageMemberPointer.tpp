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

template <class M, typename T>
MessageMemberPointer<M, T>::MessageMemberPointer(M* message, size_t offset) {
  this->impl.reset(new Impl(MessagePtr(message), offset));
}

template <class M, typename T>
MessageMemberPointer<M, T>::MessageMemberPointer(const MessagePtr&
    message, size_t offset) {
  this->impl.reset(new Impl(message, offset));
}

template <class M, typename T>
MessageMemberPointer<M, T>::MessageMemberPointer(const
    MessageMemberPointer<M, T>& src) :
  Pointer<T>(src) {
}

template <class M, typename T>
MessageMemberPointer<M, T>::MessageMemberPointer(const Pointer<T>& src) :
  Pointer<T>(src) {
  if (this->impl)
    BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(this->impl));
}

template <class M, typename T>
MessageMemberPointer<M, T>::~MessageMemberPointer() {
}

template <class M, typename T>
MessageMemberPointer<M, T>::Impl::Impl(const MessagePtr& message,
    size_t offset) :
  message(message),
  offset(offset) {
}

template <class M, typename T>
MessageMemberPointer<M, T>::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <class M, typename T>
void MessageMemberPointer<M, T>::setMessage(const MessagePtr& message) {
  boost::static_pointer_cast<Impl>(this->impl)->message = message;
}

template <class M, typename T>
const typename MessageMemberPointer<M, T>::MessagePtr&
    MessageMemberPointer<M, T>::getMessage() const {
  return boost::static_pointer_cast<Impl>(this->impl)->message;
}

template <class M, typename T>
void MessageMemberPointer<M, T>::setOffset(size_t offset) {
  boost::static_pointer_cast<Impl>(this->impl)->offset = offset;
}

template <class M, typename T>
size_t MessageMemberPointer<M, T>::getOffset() const {
  return boost::static_pointer_cast<Impl>(this->impl)->offset;
}

template <class M, typename T>
T* MessageMemberPointer<M, T>::Impl::get() const {
  if (this->message)
    return reinterpret_cast<T*>(
      (reinterpret_cast<size_t>(this->message.get())+this->offset));
  else
    return 0;
}

}
