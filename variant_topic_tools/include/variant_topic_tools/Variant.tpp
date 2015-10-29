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

#include <typeinfo>

#include <variant_topic_tools/Exceptions.h>

namespace variant_topic_tools {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T> Variant::Variant(const T& src) {
  Variant::template assign<T>(*this, src);
}

template <typename T>
Variant::ValueT<T>::ValueT() {
}

template <typename T>
Variant::ValueT<T>::~ValueT() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T> void Variant::setValue(const T& value) {
  Variant::template setValue<T>(*this, value);
}

template <typename T> typename type_traits::DataType<T>::ValueType& 
    Variant::getValue() {
  typedef typename type_traits::DataType<T>::ValueType ValueType;
  
  if (!this->type.isValid()) {
    this->type = DataType(typeid(T));
    
    if (this->type.isValid()) {
      this->value = this->type.createVariant().value;
      return boost::dynamic_pointer_cast<ValueT<ValueType> >(this->value)->
        getValue();
    }
    else
      throw InvalidDataTypeException();
  }
  else if (typeid(T) == this->type.getTypeInfo()) {
    if (!this->value)
      this->value = this->type.createVariant().value;
    
    return boost::dynamic_pointer_cast<ValueT<ValueType> >(this->value)->
      getValue();
  }
  else
    throw DataTypeMismatchException(this->type.getIdentifier(),
      DataType(typeid(T)).getIdentifier());
}

template <typename T> const typename type_traits::DataType<T>::ValueType&
    Variant::getValue() const {
  typedef typename type_traits::DataType<T>::ValueType ValueType;
  
  if (this->type.isValid()) {
    if (typeid(T) == this->type.getTypeInfo()) {
      if (!this->value) {
        static typename type_traits::DataType<T>::ValueType value;
        return value;
      }
      else 
        return boost::dynamic_pointer_cast<ValueT<ValueType> >(this->value)->
          getValue();
    }
    else
      throw DataTypeMismatchException(type.getIdentifier(),
        DataType(typeid(T)).getIdentifier());
  }
  else
    throw InvalidDataTypeException();
}

template <typename T>
const std::type_info& Variant::ValueT<T>::getTypeInfo() const {
  return typeid(T);
}

template <typename T>
void Variant::ValueT<T>::setValue(const T& value) {
  this->getValue() = value;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> void Variant::set(Variant& variant, const
    Pointer<typename type_traits::DataType<T>::ValueType>& value) {
  typedef typename type_traits::DataType<T>::ValueType ValueType;
  
  if (!variant.type.isValid()) {
    variant.type = DataType(typeid(T));
    
    if (variant.type.isValid()) {
      variant.value = variant.type.createVariant().value;
      boost::dynamic_pointer_cast<ValueT<ValueType> >(variant.value)->
        set(value);      
    }
    else
      throw InvalidDataTypeException();
  }
  else if (typeid(T) == variant.type.getTypeInfo()) {
    if (!variant.value)
      variant.value = variant.type.createVariant().value;
    
    boost::dynamic_pointer_cast<ValueT<ValueType> >(variant.value)->
      set(value);      
  }
  else
    throw DataTypeMismatchException(variant.type.getIdentifier(),
      DataType(typeid(T)).getIdentifier());
}

template <typename T> void Variant::setValue(Variant& dst, const T& value,
    typename boost::enable_if<boost::is_base_of<Variant, T> >::type*) {
  if (!dst.type.isValid()) {
    dst.type = value.type;
    
    if (dst.type.isValid()) {
      dst.value = dst.type.createVariant().value;
      if (value.value)
        dst.value->setValue(*value.value);
    }
    else
      throw InvalidDataTypeException();
  }
  else if (value.type == dst.type) {
    if (!dst.value)
      dst.value = dst.type.createVariant().value;
    
    if (value.value)
      dst.value->setValue(*value.value);
  }
  else
    throw DataTypeMismatchException(dst.type.getIdentifier(),
      value.type.getIdentifier());
}

template <typename T> void Variant::setValue(Variant& dst, const T& value,
    typename boost::disable_if<boost::is_base_of<Variant, T> >::type*) {
  typedef typename type_traits::ToDataType<T>::DataType Type;
  typedef typename type_traits::DataType<Type>::ValueType ValueType;
  
  if (!dst.type.isValid()) {
    dst.type = DataType(typeid(Type));
    
    if (dst.type.isValid()) {
      dst.value = dst.type.createVariant().value;
      boost::dynamic_pointer_cast<ValueT<ValueType> >(dst.value)->
        setValue(value);
    }
    else
      throw InvalidDataTypeException();
  }
  else if (typeid(Type) == dst.type.getTypeInfo()) {
    if (!dst.value)
      dst.value = dst.type.createVariant().value;
    
    boost::dynamic_pointer_cast<ValueT<ValueType> >(dst.value)->
      setValue(value);
  }
  else
    throw DataTypeMismatchException(dst.type.getIdentifier(),
      DataType(typeid(Type)).getIdentifier());
}
    
template <typename T> void Variant::assign(Variant& dst, const T& src,
    typename boost::enable_if<boost::is_base_of<Variant, T> >::type*) {
  dst.type = src.type;
  dst.value = src.value;
}

template <typename T> void Variant::assign(Variant& dst, const T& src,
    typename boost::disable_if<boost::is_base_of<Variant, T> >::type*) {
  dst.template setValue<T>(src);
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

template <typename T> Variant& Variant::operator=(const T& src) {
  Variant::template assign<T>(*this, src);
  return *this;
}

template <typename T> Variant::operator T() const {
  return this->template getValue<T>();
}

}
