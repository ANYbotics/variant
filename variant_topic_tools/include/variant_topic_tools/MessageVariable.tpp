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

#include <variant_topic_tools/DataTypeRegistry.h>
#include <variant_topic_tools/MessageMemberPointer.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
MessageVariable::ImplT<T>::ImplT(const std::string& name, const DataType& type)
    : Impl(name, type){}

template <typename T>
MessageVariable::ImplT<T>::ImplT::~ImplT() = default;

template <typename T, typename M>
MessageVariable::ImplM<T, M>::ImplM(const std::string& name, const DataType& type, size_t offset)
    : ImplT<T>(name, type),
      offset(offset){}

template <typename T, typename M>
MessageVariable::ImplM<T, M>::ImplM::~ImplM() = default;

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T, typename M>
MessageVariable MessageVariable::create(const std::string& name, size_t offset) {
  DataTypeRegistry registry;
  MessageVariable messageVariable;

  messageVariable.impl.reset(new ImplM<T, M>(name, registry.template getDataType<M>(), offset));

  return messageVariable;
}

template <typename T, typename M>
Variant MessageVariable::ImplM<T, M>::createVariant(const Pointer<ValueType>& message) const {
  Variant variant = this->type.createVariant();

  Variant::template set<M>(variant, MessageMemberPointer<T, M>(message, this->offset));

  return variant;
}

}  // namespace variant_topic_tools
