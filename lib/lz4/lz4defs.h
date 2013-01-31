/*
 * LZ4 Decompressor for Linux kernel
 *
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
 *     * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
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

/*
 * Detects 64 bits mode
 */
#if (defined(__x86_64__) || defined(__x86_64) || defined(__amd64__) \
	|| defined(__ppc64__) || defined(__LP64__))
#define LZ4_ARCH64 1
#else
#define LZ4_ARCH64 0
#endif

/*
 * Compiler Options
 */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L /* C99 */
/* "restrict" is a known keyword */
#else
#define restrict	/* Disable restrict */
#endif

/*
 * Architecture-specific macros
 */
#define BYTE	u8
#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
typedef struct _U32_S { u32 v; } U32_S;
typedef struct _U64_S { u64 v; } U64_S;

#define A32(x) (((U32_S *)(x))->v)
#define A64(x) (((U64_S *)(x))->v)

#define PUT4(s, d) (A32(d) = A32(s))
#define PUT8(s, d) (A64(d) = A64(s))
#else /* CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS */

#define PUT4(s, d) \
	put_unaligned(get_unaligned((const u32 *) s), (u32 *) d)
#define PUT8(s, d) \
	put_unaligned(get_unaligned((const u64 *) s), (u64 *) d)
#endif

#define COPYLENGTH 8
#define ML_BITS  4
#define ML_MASK  ((1U << ML_BITS) - 1)
#define RUN_BITS (8 - ML_BITS)
#define RUN_MASK ((1U << RUN_BITS) - 1)

#if LZ4_ARCH64/* 64-bit */
#define STEPSIZE 8

#define LZ4_COPYSTEP(s, d)	\
	do {	\
		PUT8(s, d);	\
		d += 8;	\
		s += 8;	\
	} while (0)

#define LZ4_COPYPACKET(s, d)	LZ4_COPYSTEP(s, d)

#define LZ4_SECURECOPY(s, d, e)	\
	do {				\
		if (d < e) {		\
			LZ4_WILDCOPY(s, d, e);	\
		}	\
	} while (0)

#else	/* 32-bit */
#define STEPSIZE 4

#define LZ4_COPYSTEP(s, d)	\
	do {	\
		PUT4(s, d);	\
		d += 4;	\
		s += 4;	\
	} while (0)

#define LZ4_COPYPACKET(s, d)	\
	do {			\
		LZ4_COPYSTEP(s, d);	\
		LZ4_COPYSTEP(s, d);	\
	} while (0)

#define LZ4_SECURECOPY	LZ4_WILDCOPY
#endif

#define LZ4_READ_LITTLEENDIAN_16(d, s, p) \
	(d = s - get_unaligned_le16(p))
#define LZ4_WILDCOPY(s, d, e)	\
	do {				\
		LZ4_COPYPACKET(s, d);	\
	} while (d < e)
