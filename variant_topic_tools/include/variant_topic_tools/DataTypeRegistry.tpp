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

template <typename T> DataType DataTypeRegistry::getDataType() const {
  return this->getDataType(typeid(T));
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class A> ArrayDataType DataTypeRegistry::addArrayDataType() {
  ArrayDataType arrayDataType = ArrayDataType::template create<A>();
  this->addDataType(arrayDataType);
  
  return arrayDataType;
}

template <typename T, size_t N> ArrayDataType
  DataTypeRegistry::addArrayDataType() {
  ArrayDataType arrayDataType = ArrayDataType::template create<T, N>();
  this->addDataType(arrayDataType);
  
  return arrayDataType;
}

template <typename T> BuiltinDataType DataTypeRegistry::addBuiltinDataType(
    const std::string& identifier) {
  BuiltinDataType builtinDataType = BuiltinDataType::template
    create<T>(identifier);
  this->addDataType(builtinDataType);
  
  return builtinDataType;
}

template <typename T> MessageDataType DataTypeRegistry::addMessageDataType() {
  MessageDataType messageDataType = MessageDataType::template create<T>();
  this->addDataType(messageDataType);
  
  return messageDataType;
}

}
