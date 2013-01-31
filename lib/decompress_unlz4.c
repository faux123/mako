/*
 * LZ4 decompressor for the Linux kernel.
 *
 * Linux kernel adaptation:
 * Copyright (C) 2013, LG Electronics, Kyungsik Lee <kyungsik.lee@lge.com>
 *
 * Based on LZ4 implementation by Yann Collet.
 *
 * LZ4 - Fast LZ compression algorithm
 * Copyright (C) 2011-2012, Yann Collet.
 * BSD 2-Clause License (http://www.opensource.org/licenses/bsd-license.php)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You can contact the author at :
 *  - LZ4 homepage : http://fastcompression.blogspot.com/p/lz4.html
 *  - LZ4 source repository : http://code.google.com/p/lz4/
 */

#ifdef STATIC
#define PREBOOT
#include "lz4/lz4_decompress.c"
#else
#include <linux/decompress/unlz4.h>
#endif

#include <linux/types.h>
#include <linux/lz4.h>
#include <linux/decompress/mm.h>

#include <linux/compiler.h>
#include <asm/unaligned.h>


#define LZ4_CHUNK_SIZE (8<<20)
#define ARCHIVE_MAGICNUMBER 0x184C2102

STATIC inline int INIT unlz4(u8 *input, int in_len,
				int (*fill) (void *, unsigned int),
				int (*flush) (void *, unsigned int),
				u8 *output, int *posp,
				void (*error) (char *x))
{
	int ret = -1;
	u32 chunksize = 0;
	u8 *inp;
	u8 *inp_start;
	u8 *outp;
	int size = in_len;
	size_t dest_len;


	if (output) {
		outp = output;
	} else if (!flush) {
		error("NULL output pointer and no flush function provided");
		goto exit_0;
	} else {
		outp = large_malloc(LZ4_CHUNK_SIZE);
		if (!outp) {
			error("Could not allocate output buffer");
			goto exit_0;
		}
	}

	if (input && fill) {
		error("Both input pointer and fill function provided,");
		goto exit_1;
	} else if (input) {
		inp = input;
	} else if (!fill) {
		error("NULL input pointer and missing fill function");
		goto exit_1;
	} else {
		inp = large_malloc(LZ4_COMPRESSBOUND(LZ4_CHUNK_SIZE));
		if (!inp) {
			error("Could not allocate input buffer");
			goto exit_1;
		}
	}
	inp_start = inp;

	if (posp)
		*posp = 0;

	if (fill)
		fill(inp, 4);

	chunksize = get_unaligned_le32(inp);
	if (chunksize == ARCHIVE_MAGICNUMBER) {
		inp += 4;
		size -= 4;
	} else {
		error("invalid header");
		goto exit_2;
	}

	if (posp)
		*posp += 4;

	for (;;) {

		if (fill)
			fill(inp, 4);

		chunksize = get_unaligned_le32(inp);
		if (chunksize == ARCHIVE_MAGICNUMBER) {
			inp += 4;
			size -= 4;
			if (posp)
				*posp += 4;
			continue;
		}
		inp += 4;
		size -= 4;

		if (posp)
			*posp += 4;

		if (fill) {
			if (chunksize > LZ4_COMPRESSBOUND(LZ4_CHUNK_SIZE)) {
				error("chunk length is longer than allocated");
				goto exit_2;
			}
			fill(inp, chunksize);
		}
		dest_len = LZ4_CHUNK_SIZE;
		ret = lz4_decompress(inp, chunksize, outp, &dest_len);
		if (ret < 0) {
			error("Decoding failed");
			goto exit_2;
		}

		if (flush && flush(outp, dest_len) != dest_len)
			goto exit_2;
		if (output)
			outp += dest_len;
		if (posp)
			*posp += chunksize;

		inp += chunksize;
		size -= chunksize;

		if (size == 0)
			break;
		else if (size < 0) {
			error("data corrupted");
			goto exit_2;
		}

		if (fill)
			inp = inp_start;
	}

	ret = 0;
exit_2:
	if (!input)
		large_free(inp_start);
exit_1:
	if (!output)
		large_free(outp);

exit_0:
	return ret;
}

#ifdef PREBOOT
STATIC int INIT decompress(unsigned char *buf, int in_len,
			      int(*fill)(void*, unsigned int),
			      int(*flush)(void*, unsigned int),
			      unsigned char *output,
			      int *posp,
			      void(*error)(char *x)
	)
{
	return unlz4(buf, in_len - 4, fill, flush, output, posp, error);
}
#endif
