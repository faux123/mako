/*
 * Wrapper for decompressing LZ4-compressed kernel, initramfs, and initrd
 *
 * Copyright (C) 2013, LG Electronics, Kyungsik Lee <kyungsik.lee@lge.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2,
 * or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
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
	size_t chunksize = 0;
	u8 *inp;
	u8 *inp_start;
	u8 *outp;
	int size = in_len;
#ifdef PREBOOT
	size_t out_len = get_unaligned_le32(input + in_len);
#endif
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
#ifdef PREBOOT
		if (out_len >= LZ4_CHUNK_SIZE) {
			dest_len = LZ4_CHUNK_SIZE;
			out_len -= dest_len;
		} else
			dest_len = out_len;
		ret = lz4_decompress(inp, &chunksize, outp, dest_len);
#else
		dest_len = LZ4_CHUNK_SIZE;
		ret = lz4_decompress_unknownoutputsize(inp, chunksize, outp,
				&dest_len);
#endif
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

		size -= chunksize;

		if (size == 0)
			break;
		else if (size < 0) {
			error("data corrupted");
			goto exit_2;
		}

		inp += chunksize;
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
