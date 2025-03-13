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

template <typename T, typename M>
MessageMemberPointer<T, M>::MessageMemberPointer(ValueType* message, size_t offset) {
  this->impl.reset(new Impl(Pointer<ValueType>(message), offset));
}

template <typename T, typename M>
MessageMemberPointer<T, M>::MessageMemberPointer(const Pointer<ValueType>& message, size_t offset) {
  this->impl.reset(new Impl(message, offset));
}

template <typename T, typename M>
MessageMemberPointer<T, M>::MessageMemberPointer(const MessageMemberPointer<T, M>& src) : Pointer<MemberValueType>(src) {}

template <typename T, typename M>
MessageMemberPointer<T, M>::MessageMemberPointer(const Pointer<MemberValueType>& src) : Pointer<MemberValueType>(src) {
  if (this->impl) BOOST_ASSERT(boost::dynamic_pointer_cast<Impl>(this->impl));
}

template <typename T, typename M>
MessageMemberPointer<T, M>::~MessageMemberPointer() {}

template <typename T, typename M>
MessageMemberPointer<T, M>::Impl::Impl(const Pointer<ValueType>& message, size_t offset) : message(message), offset(offset) {}

template <typename T, typename M>
MessageMemberPointer<T, M>::Impl::~Impl() {}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T, typename M>
void MessageMemberPointer<T, M>::setMessage(const Pointer<ValueType>& message) {
  boost::static_pointer_cast<Impl>(this->impl)->message = message;
}

template <typename T, typename M>
const Pointer<typename MessageMemberPointer<T, M>::ValueType>& MessageMemberPointer<T, M>::getMessage() const {
  return boost::static_pointer_cast<Impl>(this->impl)->message;
}

template <typename T, typename M>
void MessageMemberPointer<T, M>::setOffset(size_t offset) {
  boost::static_pointer_cast<Impl>(this->impl)->offset = offset;
}

template <typename T, typename M>
size_t MessageMemberPointer<T, M>::getOffset() const {
  return boost::static_pointer_cast<Impl>(this->impl)->offset;
}

template <typename T, typename M>
typename MessageMemberPointer<T, M>::MemberValueType* MessageMemberPointer<T, M>::Impl::get() const {
  if (this->message)
    return reinterpret_cast<MemberValueType*>((reinterpret_cast<size_t>(this->message.get()) + this->offset));
  else
    return 0;
}

}  // namespace variant_topic_tools
