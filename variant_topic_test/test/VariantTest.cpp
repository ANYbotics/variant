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

#include <gtest/gtest.h>

#include <variant_topic_tools/Variant.h>

using namespace variant_topic_tools;

TEST(Variant, Builtin) {
  Variant variant(DataType("float64"));
  
  EXPECT_TRUE(variant.hasType());
  EXPECT_TRUE(variant.isEmpty());
  EXPECT_EQ(0.0, variant.getValue<double>());
  EXPECT_FALSE(variant.isEmpty());
  EXPECT_ANY_THROW(variant.getValue<int>());
  EXPECT_ANY_THROW(variant.setValue(0));
  
  variant = M_PI;

  EXPECT_EQ(M_PI, variant.getValue<double>());
  EXPECT_TRUE(variant == M_PI);
  EXPECT_FALSE(variant != M_PI);
  EXPECT_TRUE(variant != 1.0);
  EXPECT_FALSE(variant == 1.0);
  
  variant = 1.0;
  
  EXPECT_TRUE(variant != 1);
  EXPECT_FALSE(variant == 1);
  
  variant.clear();
  
  EXPECT_FALSE(variant.hasType());
  EXPECT_TRUE(variant.isEmpty());
}
