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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *.
 ******************************************************************************/

#include <gtest/gtest.h>

#include <variant_topic_tools/MessageTypeParser.h>

using namespace variant_topic_tools;

TEST(MessageTypeParser, Match) {
  std::string package;
  std::string type;

  EXPECT_TRUE(MessageTypeParser::matchType("PlainType", package, type));
  EXPECT_EQ(std::string(), package);
  EXPECT_EQ("PlainType", type);
  EXPECT_TRUE(MessageTypeParser::matchType("package/PackageType", package, type));
  EXPECT_EQ("package", package);
  EXPECT_EQ("PackageType", type);
}
