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

/** \file MD5Sum.h
  * \brief Header file providing the MD5Sum class interface
  */

#ifndef VARIANT_TOPIC_TOOLS_MD5_SUM_H
#define VARIANT_TOPIC_TOOLS_MD5_SUM_H

#include <boost/array.hpp>

#include <ros/ros.h>

namespace variant_topic_tools {
  /** \brief MD5 sum computation algorithm
    * 
    * This class implements the Message-Digest Algorithm 5 for computing
    * the MD5 sum of a string.
    */
  class MD5Sum {
  public:
    /** \brief Definition of the MD5 sum digest type
      */ 
    typedef boost::array<uint32_t, 4> Digest;
    
    /** \brief Default constructor
      */ 
    MD5Sum(const std::string& text = std::string());
    
    /** \brief Copy constructor
      */ 
    MD5Sum(const MD5Sum& src);
    
    /** \brief Destructor
      */ 
    ~MD5Sum();

    /** \brief Retrieve the MD5 sum's number of digested bits
      */ 
    size_t getNumDigestedBits() const;
    
    /** \brief Retrieve the MD5 sum's digest
      */ 
    Digest getDigest() const;
    
    /** \brief Update the MD5 sum using the given string
      */ 
    void update(const std::string& text);
    
    /** \brief Clear the MD5 sum
      */ 
    void clear();
    
    /** \brief Write the MD5 sum to a stream
      */
    void write(std::ostream& stream) const;
    
    /** \brief Convert the MD5 sum to a hexadecimal format string
      */ 
    std::string toString() const;
    
    /** \brief Stream operator for updating the MD5 sum
      */ 
    MD5Sum& operator<<(const std::string& text);
    
  protected:
    /** \brief Definition of the MD5 sum size type
      */ 
    typedef boost::array<uint32_t, 2> Size;
    
    /** \brief Definition of the MD5 sum block type
      */ 
    typedef boost::array<uint32_t, 16> Block;
    
    /** \brief Initial digest
      */ 
    static const Digest initialDigest;
    
    /** \brief Number of bit rotations per round according to the
      *   MD5 algorithm
      */ 
    static const boost::array<uint32_t, 64> numBitRotations;
    
    /** \brief Constants according to the MD5 algorithm
      */ 
    static const boost::array<uint32_t, 64> constants;
    
    /** \brief Padding required to fill the last MD5 block for finalization
      */ 
    static const Block padding;
    
    /** \brief The current digest
      */ 
    Digest digest;

    /** \brief The number of digested bits
      */ 
    Size numDigestedBits;
    
    /** \brief The buffered bytes which did not fit into the last block
      */ 
    Block buffer;
    
    /** \brief Update the MD5 sum using the given array of bytes
      */ 
    void update(const uint8_t* bytes, size_t numBytes);
        
    /** \brief Transform a block
      */ 
    static void transform(const Block& block, Digest& digest);
    
    /** \brief Perform bit rotation on a digest element
      */ 
    static uint32_t rotateLeft(uint32_t value, size_t numBits);
    
    /** \brief Update a digest and its context
      */ 
    static void update(Digest& digest, Size& numDigestedBits, Block& buffer,
      const uint8_t* bytes, size_t numBytes);
    
    /** \brief Compute the finalized digest
      */ 
    static void finalize(Digest& digest, Size& numDigestedBits, Block& buffer);
  };
    
  /** \brief Operator for writing the MD5 sum to a stream
    */
  std::ostream& operator<<(std::ostream& stream, const MD5Sum& md5Sum);
};

#endif
