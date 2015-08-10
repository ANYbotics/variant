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

#include <variant_topic_tools/MD5Sum.h>

using namespace variant_topic_tools;

TEST (MD5Sum, StringHashing) {
  MD5Sum h1;
  MD5Sum h2("0123456789abcdef");
  MD5Sum h3("Franz jagt im komplett verwahrlosten Taxi quer durch Bayern");
  MD5Sum h4("Frank jagt im komplett verwahrlosten Taxi quer durch Bayern");
  MD5Sum h5;
  h5 << "Franz" << " jagt" << " im" << " komplett"<< " verwahrlosten" <<
    " Taxi" << " quer" << " durch" << " Bayern";

  EXPECT_EQ(0, h1.getNumDigestedBits());
  EXPECT_EQ("d41d8cd98f00b204e9800998ecf8427e", h1.toString());
  EXPECT_EQ(16*8, h2.getNumDigestedBits());
  EXPECT_EQ("4032af8d61035123906e58e067140cc5", h2.toString());
  EXPECT_EQ(59*8, h3.getNumDigestedBits());
  EXPECT_EQ("a3cca2b2aa1e3b5b3b5aad99a8529074", h3.toString());
  EXPECT_EQ(59*8, h4.getNumDigestedBits());
  EXPECT_EQ("7e716d0e702df0505fc72e2b89467910", h4.toString());
  EXPECT_EQ(59*8, h5.getNumDigestedBits());
  EXPECT_EQ("a3cca2b2aa1e3b5b3b5aad99a8529074", h5.toString());
}
