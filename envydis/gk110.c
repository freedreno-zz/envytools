/*
 *
 * Copyright (C) 2009-2011 Marcin Kościelnicki <koriakin@0x04.net>
 * Copyright (C) 2012 Christoph Bumiller <e0425955@student.tuwien.ac.at>
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * VA LINUX SYSTEMS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "dis-intern.h"

#define F_GK110 1

/*
 * $r255: bit bucket on write, 0 on read
 */

/*
 * Code target field
 */
static struct rbitfield ctargoff = { { 23, 24 }, RBF_SIGNED, .pcrel = 1, .addend = 8 };
#define BTARG atomctarg, &ctargoff
#define CTARG atomctarg, &ctargoff


/*
 * Misc number fields
 */
static struct rbitfield fimmoff = { { 0x17, 19 }, RBF_UNSIGNED, 12 };
static struct rbitfield limmoff = { { 0x17, 32 }, .wrapok = 1 };
#define FIMM atomrimm, &fimmoff
#define LIMM atomrimm, &limmoff

/*
 * Register fields
 */
static struct sreg reg_sr[] = {
	{ 255, 0, SR_ZERO },
	{ -1 },
};
static struct sreg pred_sr[] = {
	{ 7, 0, SR_ONE },
	{ -1 },
};

static struct bitfield dst_bf = { 0x2, 8 };
static struct bitfield pdst_bf = { 0x2, 3 };
static struct bitfield src1_bf = { 0xa, 8 };
static struct bitfield src2_bf = { 0x17, 8 };
static struct bitfield src3_bf = { 0x2a, 8 };
static struct bitfield pred_bf = { 0x12, 3 };

static struct reg dst_r = { &dst_bf, "r", .specials = reg_sr };
static struct reg dstd_r = { &dst_bf, "r", "d" };
static struct reg dstq_r = { &dst_bf, "r", "q" };
static struct reg src1_r = { &src1_bf, "r", .specials = reg_sr };
static struct reg src1d_r = { &src1_bf, "r", "d" };
static struct reg src2_r = { &src2_bf, "r", .specials = reg_sr };
static struct reg src2d_r = { &src2_bf, "r", "d" };
static struct reg src3_r = { &src3_bf, "r", .specials = reg_sr };
static struct reg src3d_r = { &src3_bf, "r", "d" };
static struct reg pdst_r = { &pdst_bf, "p", .specials = pred_sr, .cool = 1 };
static struct reg pred_r = { &pred_bf, "p", .specials = pred_sr, .cool = 1 };

#define DST atomreg, &dst_r
#define DSTD atomreg, &dstd_r
#define DSTQ atomreg, &dstq_r
#define PRED atomreg, &pred_r
#define SRC1 atomreg, &src1_r
#define SRC1D atomreg, &src1d_r
#define SRC2 atomreg, &src2_r
#define SRC2D atomreg, &src2d_r
#define SRC3 atomreg, &src3_r
#define SRC3D atomreg, &src3d_r


/*
 * Memory fields
 */

static struct rbitfield cmem_imm = { { 0x17, 14 }, RBF_SIGNED, .shr = 2 };
static struct bitfield cmem_idx = { 0x25, 5 };

static struct mem cmem_m = { "c", &cmem_idx, 0, &cmem_imm };

#define CONST atommem, &cmem_m


/*
 * The instructions
 */

static struct insn tabfrm2a[] = {
	{ 0x00000000, 0x00000c00, N("rn") },
	{ 0x00000400, 0x00000c00, N("rm") },
	{ 0x00000800, 0x00000c00, N("rp") },
	{ 0x00000c00, 0x00000c00, N("rz") },
	{ 0, 0, OOPS },
};
static struct insn tabfrm36[] = {
	{ 0x00000000, 0x00c00000, N("rn") },
	{ 0x00400000, 0x00c00000, N("rm") },
	{ 0x00800000, 0x00c00000, N("rp") },
	{ 0x00c00000, 0x00c00000, N("rz") },
	{ 0, 0, OOPS },
};

static struct insn tablane2a[] = {
	{ 0x0000000000000000ull, 0x00003c0000000000ull, N("lnone") },
	{ 0x0000040000000000ull, 0x00003c0000000000ull, N("l0") },
	{ 0x0000080000000000ull, 0x00003c0000000000ull, N("l1") },
	{ 0x00000c0000000000ull, 0x00003c0000000000ull, N("l01") },
	{ 0x0000100000000000ull, 0x00003c0000000000ull, N("l2") },
	{ 0x0000140000000000ull, 0x00003c0000000000ull, N("l02") },
	{ 0x0000180000000000ull, 0x00003c0000000000ull, N("l12") },
	{ 0x00001c0000000000ull, 0x00003c0000000000ull, N("l012") },
	{ 0x0000200000000000ull, 0x00003c0000000000ull, N("l3") },
	{ 0x0000240000000000ull, 0x00003c0000000000ull, N("l03") },
	{ 0x0000280000000000ull, 0x00003c0000000000ull, N("l13") },
	{ 0x00002c0000000000ull, 0x00003c0000000000ull, N("l013") },
	{ 0x0000300000000000ull, 0x00003c0000000000ull, N("l23") },
	{ 0x0000340000000000ull, 0x00003c0000000000ull, N("l023") },
	{ 0x0000380000000000ull, 0x00003c0000000000ull, N("l123") },
	{ 0x00003c0000000000ull, 0x00003c0000000000ull },
	{ 0, 0, OOPS },
};

static struct insn tabis2[] = {
	{ 0x4000000000000000ull, 0xc000000000000000ull, CONST },
	{ 0xc000000000000000ull, 0xc000000000000000ull, SRC2 },
	{ 0, 0, OOPS },
};

static struct insn tabis2w3[] = {
	{ 0x4000000000000000ull, 0xc000000000000000ull, CONST },
	{ 0x8000000000000000ull, 0xc000000000000000ull, SRC3 },
	{ 0xc000000000000000ull, 0xc000000000000000ull, SRC2 },
	{ 0, 0, OOPS },
};

static struct insn tabis3[] = {
	{ 0x4000000000000000ull, 0xc000000000000000ull, SRC3 },
	{ 0x8000000000000000ull, 0xc000000000000000ull, CONST },
	{ 0xc000000000000000ull, 0xc000000000000000ull, SRC3 },
	{ 0, 0, OOPS },
};

F1(sat35, 0x35, N("sat")) // add f32
F1(ftz2f, 0x2f, N("ftz")) // add,mul f32
F1(ftz38, 0x38, N("ftz")) // fma f32
F1(neg30, 0x30, N("neg")) // add f32 src2
F1(neg33, 0x33, N("neg")) // add f32 src1; mul,fma f32 src2
F1(neg34, 0x34, N("neg")) // fma f32 src3
F1(abs31, 0x31, N("abs")) // add f32 src1
F1(abs34, 0x34, N("abs")) // add f32 src2

/*
 * Opcode format
 *
 * 0000000000000003 type (control, immediate, normal)
 * 00000000000003fc dst
 * 000000000003fc00 1st src
 * 00000000001c0000 predicate
 * 0000000000200000 negate predicate
 * 0000000000400000 join
 * 000000007f800000 2nd src
 * 0000003fff800000 2nd src (immediate)
 * 0000007fff800000 address
 * 007fffffff800000 long immediate
 * 0003fc0000000000 3rd src
 * 007c000000000000 misc flags
 * 1f80000000000000 opcode
 * c000000000000000 source type
 */

static struct insn tabm[] = {
	{ 0x0c00000000000002ull, 0x3fc0000000000003ull, N("fma"), T(ftz38),           T(frm36), N("f32"), DST, T(neg33), SRC1, T(is2w3), T(neg34), T(is3) },
	{ 0x22c0000000000002ull, 0x3fc0000000000003ull, N("add"), T(ftz2f), T(sat35), T(frm2a), N("f32"), DST, T(neg33), T(abs31), SRC1, T(neg30), T(abs34), T(is2) },
	{ 0x2340000000000002ull, 0x3fc0000000000003ull, N("mul"), T(ftz2f),           T(frm2a), T(neg33), N("f32"), DST, SRC1, T(is2) },
	{ 0x24c0000000000002ull, 0x3fc0000000000003ull, T(lane2a), N("mov"), N("b32"), DST, T(is2) },
	{ 0x0, 0x0, OOPS },
};

static struct insn tabi[] = {
	{ 0x02c0000000000001ull, 0x3fc0000000000003ull, N("add"), T(ftz2f), T(sat35), T(frm2a), N("f32"), DST, T(neg33), T(abs31), SRC1, T(neg30), T(abs34), T(is2) },
	{ 0x0340000000000001ull, 0x3fc0000000000003ull, N("mul"), T(ftz2f),           T(frm2a), T(neg33), N("f32"), DST, SRC1, T(is2) },
	{ 0x1400000000000001ull, 0x3fc0000000000003ull, N("fma"), T(ftz38),           T(frm36), N("f32"), DST, T(neg33), SRC1, T(is2w3), T(neg34), T(is3) },
	{ 0x0, 0x0, OOPS },
};

static struct insn tabp[] = {
	{ 0x001c0000, 0x003c0000 },
	{ 0x003c0000, 0x003c0000, N("never") },
	{ 0x00000000, 0x00200000, PRED },
	{ 0x00200000, 0x00200000, SESTART, N("not"), PRED, SEEND },
	{ 0, 0, OOPS },
};

static struct insn tabc[] = {
	{ 0x1200000000000000ull, 0x1f80000000000000ull, T(p), N("bra"), BTARG },
	{ 0x1480000000000000ull, 0x1f80000000000000ull, N("joinat"), BTARG },
	{ 0, 0, OOPS },
};

static struct insn tabroot[] = {
	// control instructions
	{ 0x00000000, 0x00400003, OP8B, T(c) },
	// short immediate (fugly)
	{ 0x00000001, 0x00400003, OP8B, T(p), T(i) },
	{ 0x00400001, 0x00400003, OP8B, N("join"), T(p), T(i) },
	// normal
	{ 0x00000002, 0x00400003, OP8B, T(p), T(m) },
	{ 0x00400002, 0x00400003, OP8B, N("join"), T(p), T(m) },
	{ 0, 0, OOPS },
};

static void gk110_prep(struct disisa *isa) {
	// no variants yet
}

struct disisa gk110_isa_s = {
	tabroot,
	8,
	4,
	1,
	.i_need_nv50as_hack = 1,
	.prep = gk110_prep,
};
