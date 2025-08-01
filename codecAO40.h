/*    AO40 encoder / decoder
 *    Copyright 2002 Phil Karn, KA9Q
 *    May be used under the terms of the GNU General Public License (GPL) 
 *    See CodecAO40.cpp for lineage
 *
 *    This file is part of FUNcubeLib.
 *
 *    FUNcubeLib is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    FUNcubeLib is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with FUNcubeLib If not, see <http://www.gnu.org/licenses/>.
 *
*/

#pragma once

//#include "fecConstants.h"


/*
	Amsat P3 FEC Encoder/decoder system. Look-up tables
	Created by Phil Karn KA9Q and James Miller G3RUH
	Last modified 2003 Jun 20  
*/

/* Defines for Viterbi Decoder for r=1/2 k=7  (to CCSDS convention) */
#define K 7                      /* Constraint length */
#define N 2                      /* Number of symbols per data bit */
#define CPOLYA 0x4f              /* First  convolutional encoder polynomial */
#define CPOLYB 0x6d              /* Second convolutional encoder polynomial */

#define SYNC_POLY   0x48         /* Sync vector PN polynomial */

#define NN        255
#define KK        223
#define NROOTS     32            /* NN-KK */
#define A0        (NN)
#define FCR       112
#define PRIM       11
#define IPRIM     116
#define BLOCKSIZE 256            /* Data bytes per frame */
#define RSBLOCKS    2            /* Number of RS decoder blocks */
#define RSPAD      95            /* Unused bytes in block  (KK-BLOCKSIZE/RSBLOCKS) */

/* Defines for Interleaver */
#define ROWS       80            /* Block interleaver rows */
#define COLUMNS    65            /* Block interleaver columns */
#define SYMPBLOCK (ROWS*COLUMNS) /* Encoded symbols per block */

/* Number of symbols in an FEC block that are */
/* passed to the Viterbi decoder  (320*8 + 6) */
#define NBITS ((BLOCKSIZE+NROOTS*RSBLOCKS)*8+K-1)
/* Number of bits obtained from Viterbi decoder */
#define NBITS_OUT (BLOCKSIZE+NROOTS*RSBLOCKS)


extern unsigned char m_RS_block[RSBLOCKS][NROOTS]; /* RS parity blocks */
extern     unsigned char m_encoded[SYMPBLOCK] ;       /* encoded symbols */
extern     int m_encoded_bytes;               /* Byte counter for encode_data() */
extern     int m_ileaver_index;               /* Byte counter for interleaver */
extern     unsigned char m_conv_sr;           /* Convolutional encoder shift register state */

extern const unsigned char RS_poly[];
extern const unsigned char ALPHA_TO[];
extern const unsigned char INDEX_OF[];
extern const unsigned char Partab[];
extern const unsigned char Scrambler[];

//class CCodecAO40
//{
//public:
//	CCodecAO40(void);
//    virtual ~CCodecAO40(void);

    int decode(unsigned char viterbi_decoded[NBITS_OUT], unsigned char *decoded_data);

    /* Encodes the 256 byte source block into 5200 byte block of symbols into m_encoded buffer */
     const unsigned char *encode(
        unsigned char *source_bytes,  /* input to encode */    
        int byte_count);              /* input length in bytes */
    
    /* Compares raw input symbols to current buffer of encoded symbols and counts the errors */            
    int count_errors( unsigned char *raw_symbols);        

//private:
    int mod255(int x);
    int decode_rs_8(char *data, int *eras_pos, int no_eras);
    void scramble_and_encode(unsigned char c);
    void encode_and_interleave(unsigned char c,int cnt);

    void descramble_to_rsblocks(
        unsigned char viterbi_decoded[NBITS_OUT],
        char rsblocks[RSBLOCKS][NN]);

    void init_encoder(void);
    void encode_byte(unsigned char c);
    void encode_parity(void);
    
    void interleave_symbol(int c);


//};
