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

#include "variant_topic_tools/DataTypeRegistry.h"
#include "variant_topic_tools/Exceptions.h"
#include "variant_topic_tools/MessageDefinitionParser.h"

namespace variant_topic_tools {

/*****************************************************************************/
/* Static initializations                                                    */
/*****************************************************************************/

DataTypeRegistry::ImplPtr DataTypeRegistry::impl(new DataTypeRegistry::Impl());

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

DataTypeRegistry::DataTypeRegistry() {
  if (impl->dataTypesByIdentifier.empty()) {
    addBuiltinDataType<bool>("bool");
    addBuiltinDataType<double>("float64");
    addBuiltinDataType<float>("float32");
    addBuiltinDataType<int16_t>("int16");
    addBuiltinDataType<int32_t>("int32");
    addBuiltinDataType<int64_t>("int64");
    addBuiltinDataType<int8_t>("int8");
    addBuiltinDataType<uint16_t>("uint16");
    addBuiltinDataType<uint32_t>("uint32");
    addBuiltinDataType<uint64_t>("uint64");
    addBuiltinDataType<uint8_t>("uint8");
    
    addBuiltinDataType<uint8_t>("char");
    addBuiltinDataType<int8_t>("byte");
    
    addBuiltinDataType<ros::Duration>("duration");
    addBuiltinDataType<std::string>("string");
    addBuiltinDataType<ros::Time>("time");
  }
}

DataTypeRegistry::~DataTypeRegistry() {
}

DataTypeRegistry::Impl::Impl() {
}

DataTypeRegistry::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

DataType DataTypeRegistry::getDataType(const std::string& identifier) {
  boost::unordered_map<std::string, DataType>::const_iterator it =
    impl->dataTypesByIdentifier.find(identifier);
  
  if (it != impl->dataTypesByIdentifier.end())
    return it->second;
  
  std::string name, memberType;
  size_t size;
  
  if (MessageDefinitionParser::matchArrayType(identifier, memberType,
      size)) {
    boost::unordered_map<std::string, DataType>::const_iterator jt =
      impl->dataTypesByIdentifier.find(memberType);
    
    if (jt != impl->dataTypesByIdentifier.end())
      return addArrayDataType(jt->second, size);
  }
  
  return DataType();
}

DataType DataTypeRegistry::getDataType(const std::string& identifier) const {
  boost::unordered_map<std::string, DataType>::const_iterator it =
    impl->dataTypesByIdentifier.find(identifier);
  
  if (it != impl->dataTypesByIdentifier.end())
    return it->second;
  else
    return DataType();
}

DataType DataTypeRegistry::getDataType(const std::type_info& typeInfo) const {
  boost::unordered_multimap<const std::type_info*, DataType, TypeInfoHash>::
    const_iterator it = impl->dataTypesByInfo.find(&typeInfo);
    
  if (it != impl->dataTypesByInfo.end())
    return it->second;
  else
    return DataType();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

ArrayDataType DataTypeRegistry::addArrayDataType(const DataType& memberType,
    size_t numMembers) {
  ArrayDataType arrayDataType(memberType, numMembers);
  addDataType(arrayDataType);
  
  return arrayDataType;
}

MessageDataType DataTypeRegistry::addMessageDataType(const std::string&
    identifier, const std::vector<MessageConstant>& constantMembers, const
    std::vector<MessageVariable>& variableMembers) {
  MessageDataType messageDataType(identifier, constantMembers,
    variableMembers);
  addDataType(messageDataType);
  
  return messageDataType;
}

MessageDataType DataTypeRegistry::addMessageDataType(const std::string&
    identifier, const std::string& definition) {
  MessageDataType messageDataType(identifier, definition);
  addDataType(messageDataType);
  
  return messageDataType;
}

void DataTypeRegistry::addDataType(const DataType& dataType) {
  if (dataType.isValid()) {
    boost::unordered_map<std::string, DataType>::iterator it =
      impl->dataTypesByIdentifier.find(dataType.getIdentifier());
      
    if (it == impl->dataTypesByIdentifier.end()) {
      impl->dataTypesByIdentifier.insert(
        std::make_pair(dataType.getIdentifier(), dataType));
      
      if (dataType.hasTypeInfo())
        impl->dataTypesByInfo.insert(
          std::make_pair(&dataType.getTypeInfo(), dataType));
    }
    else if (!it->second.hasTypeInfo() && dataType.hasTypeInfo()) {
      it->second = dataType;
      
      impl->dataTypesByInfo.insert(
        std::make_pair(&dataType.getTypeInfo(), dataType));
    }
    else
      throw AmbiguousDataTypeIdentifierException(it->first);
  }
  else
    throw InvalidDataTypeException();
}

void DataTypeRegistry::removeDataType(const DataType& dataType) {
  if (dataType.isValid()) {
    boost::unordered_map<std::string, DataType>::iterator it =
      impl->dataTypesByIdentifier.find(dataType.getIdentifier());
      
    if ((it != impl->dataTypesByIdentifier.end()) &&
        (it->second.impl == dataType.impl))
      impl->dataTypesByIdentifier.erase(it);
    
    if (dataType.hasTypeInfo()) {
      typedef boost::unordered_multimap<const std::type_info*, DataType,
        TypeInfoHash>::iterator Iterator;
        
      std::pair<Iterator, Iterator> range =
        impl->dataTypesByInfo.equal_range(&dataType.getTypeInfo());
      for (Iterator it = range.first; it != range.second; ++it) {
        if (it->second.impl == dataType.impl) {
          impl->dataTypesByInfo.erase(it);
          break;
        }
      }
    }
  }
  else
    throw InvalidDataTypeException();
}

void DataTypeRegistry::clear() {
  impl->dataTypesByIdentifier.clear();
  impl->dataTypesByInfo.clear();
}

void DataTypeRegistry::write(std::ostream& stream) const {
  for (boost::unordered_map<std::string, DataType>::const_iterator
      it = impl->dataTypesByIdentifier.begin();
      it != impl->dataTypesByIdentifier.end(); ++it) {
    if (it != impl->dataTypesByIdentifier.begin())
      stream << "\n";
    stream << it->second;
  }
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

DataType DataTypeRegistry::operator[](const std::string& identifier) {
  return getDataType(identifier);
}

DataType DataTypeRegistry::operator[](const std::string& identifier) const {
  return getDataType(identifier);
}

DataType DataTypeRegistry::operator[](const std::type_info& typeInfo) const {
  return getDataType(typeInfo);
}

std::ostream& operator<<(std::ostream& stream, const DataTypeRegistry&
    dataTypeRegistry) {
  dataTypeRegistry.write(stream);
  return stream;
}

}
