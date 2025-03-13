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

#include <variant_topic_tools/MessageFieldCollection.h>

using namespace variant_topic_tools;

TEST(MessageFieldCollection, Namespaces) {
  variant_topic_tools::MessageFieldCollection<bool> c1;

  c1.appendField("field1");
  c1.appendField("field2", true);
  c1["field2"].appendField("field2_1");

  EXPECT_FALSE(c1.isEmpty());
  EXPECT_EQ(2, c1.getNumFields());

  EXPECT_TRUE(c1.hasField("field1"));
  EXPECT_TRUE(c1.hasField("/field2"));
  EXPECT_TRUE(c1.hasField("field2/field2_1"));
  EXPECT_TRUE(c1.hasField("/field2/field2_1"));
  EXPECT_FALSE(c1.hasField("field3"));
  EXPECT_FALSE(c1.hasField("field2/field2_2"));

  EXPECT_NO_THROW(c1.getField("field1"));
  EXPECT_NO_THROW(c1.getField("/field2"));
  EXPECT_NO_THROW(c1.getField("field2/field2_1"));
  EXPECT_NO_THROW(c1.getField("/field2/field2_1"));
  EXPECT_ANY_THROW(c1.getField("field3"));
  EXPECT_ANY_THROW(c1.getField("field2/field2_2"));

  EXPECT_FALSE(c1["field1"].getValue());
  EXPECT_TRUE(c1["field2"].getValue());

  c1.clear();

  EXPECT_TRUE(c1.isEmpty());
}
