#ifndef TARGET_RISCV_OPCODES_H_
#define TARGET_RISCV_OPCODES_H_

#include "encoding.h"
#include <stdint.h>
#include <assert.h>
#include <limits.h>

static inline uint32_t
__attribute__((const))
bits(uint32_t const value,
	unsigned const hi,
	unsigned const lo)
{
	assert(lo <= hi && hi < CHAR_BIT * sizeof(uint32_t));
	return (value >> lo) & ((1 << (hi + 1 - lo)) - 1);
}

static inline uint32_t
__attribute__((const))
bit(uint32_t const value,
	unsigned const b)
{
	assert(b < CHAR_BIT * sizeof(uint32_t));
	return 1 & (value >> b);
}

static inline uint32_t
__attribute__((const))
sw(unsigned const src,
	unsigned const base,
	uint16_t const offset)
{
	assert(src < 32 && base < 32);
	return
		bits(offset, 11, 5) << 25 |
		src << 20 |
		base << 15 |
		bits(offset, 4, 0) << 7 |
		MATCH_SW;
}

static inline uint32_t
__attribute__((const))
sh(unsigned const src,
	unsigned const base,
	uint16_t const offset)
{
	assert(src < 32 && base < 32);
	return
		bits(offset, 11, 5) << 25 |
		src << 20 |
		base << 15 |
		bits(offset, 4, 0) << 7 |
		MATCH_SH;
}

static inline uint32_t
__attribute__((const))
sb(unsigned const src,
	unsigned const base,
	uint16_t const offset)
{
	assert(src < 32 && base < 32);
	return
		bits(offset, 11, 5) << 25 |
		src << 20 |
		base << 15 |
		bits(offset, 4, 0) << 7 |
		MATCH_SB;
}


/**
@bug @c offset should be signed
*/
static inline uint32_t
__attribute__((const))
lw(unsigned const rd,
	unsigned const base,
	uint16_t const offset)
{
	assert(rd < 32 && base < 32);
	return
		bits(offset, 11, 0) << 20 |
		base << 15 |
		bits(rd, 4, 0) << 7 |
		MATCH_LW;
}

/**
@bug @c offset should be signed
*/
static inline uint32_t
__attribute__((const))
lh(unsigned const rd,
	unsigned const base,
	uint16_t const offset)
{
	assert(rd < 32 && base < 32);
	return
		bits(offset, 11, 0) << 20 |
		base << 15 |
		bits(rd, 4, 0) << 7 |
		MATCH_LH;
}

/**
@bug @c offset should be signed
*/
static inline uint32_t
__attribute__((const))
lb(unsigned const rd,
	unsigned const base,
	uint16_t const offset)
{
	assert(base < 32 && rd < 32);
	return
		bits(offset, 11, 0) << 20 |
		base << 15 |
		bits(rd, 4, 0) << 7 |
		MATCH_LB;
}

static inline uint32_t
__attribute__((const))
addi(unsigned const dest,
	unsigned const src,
	uint16_t const imm)
{
	assert(src < 32 && dest < 32);
	return
		bits(imm, 11, 0) << 20 |
		src << 15 |
		dest << 7 |
		MATCH_ADDI;
}

static inline uint32_t
__attribute__((const))
csrrs(unsigned const rd,
	unsigned const rs,
	unsigned const csr)
{
	assert(rs < 32 && rd < 32);
	return
		csr << 20 |
		rs << 15 |
		rd << 7 |
		MATCH_CSRRS;
}

static inline uint32_t
__attribute__((const))
csrrw(unsigned const rd,
	unsigned const rs,
	unsigned const csr)
{
	assert(rs < 32 && rd < 32);
	return
		csr << 20 |
		rs << 15 |
		rd << 7 |
		MATCH_CSRRW;
}

static inline uint32_t
__attribute__((const))
fsd(unsigned const src,
	unsigned const base,
	uint16_t const offset)
{
	assert(src < 32 && base < 32);
	return
		bits(offset, 11, 5) << 25 |
		bits(src, 4, 0) << 20 |
		base << 15 |
		bits(offset, 4, 0) << 7 |
		MATCH_FSD;
}

static inline uint32_t
__attribute__((const))
fld(unsigned const dest,
	unsigned const base,
	uint16_t const offset)
{
	assert(dest < 32 && base < 32);
	return
		bits(offset, 11, 0) << 20 |
		base << 15 |
		bits(dest, 4, 0) << 7 |
		MATCH_FLD;
}

static inline uint32_t
__attribute__((const))
fmv_x_w(unsigned const dest,
	unsigned const src)
{
	assert(src < 32 && dest < 32);
	return
		src << 15 |
		dest << 7 |
		MATCH_FMV_X_W;
}

static inline uint32_t
__attribute__((const))
fmv_x_d(unsigned const dest,
	unsigned const src)
{
	assert(src < 32 && dest < 32);
	return
		src << 15 |
		dest << 7 |
		MATCH_FMV_X_D;
}

static inline uint32_t
__attribute__((const))
fmv_w_x(unsigned const dest,
	unsigned const src)
{
	assert(src < 32 && dest < 32);
	return
		src << 15 |
		dest << 7 |
		MATCH_FMV_W_X;
}

static inline uint32_t
__attribute__((const))
fmv_d_x(unsigned const dest,
	unsigned const src)
{
	assert(src < 32 && dest < 32);
	return
		src << 15 |
		dest << 7 |
		MATCH_FMV_D_X;
}

static inline uint32_t
__attribute__((const))
ebreak(void)
{
	return MATCH_EBREAK;
}

static inline uint16_t
__attribute__((const))
ebreak_c(void)
{
	return MATCH_C_EBREAK;
}

static inline uint32_t
__attribute__((const))
wfi(void)
{
	return MATCH_WFI;
}

static inline uint32_t
__attribute__((const))
fence_i(void)
{
	return MATCH_FENCE_I;
}

/**
@bug @c imm should be signed
*/
static inline uint32_t
__attribute__((const))
lui(unsigned const dest,
	uint32_t const imm)
{
	assert(dest < 32);
	return
		bits(imm, 19, 0) << 12 |
		dest << 7 |
		MATCH_LUI;
}

static inline uint32_t
__attribute__((const))
xori(unsigned const dest,
	unsigned const src,
	uint16_t const imm)
{
	assert(src < 32 && dest < 32);
	return
		bits(imm, 11, 0) << 20 |
		src << 15 |
		dest << 7 |
		MATCH_XORI;
}

static inline uint32_t
__attribute__((const))
srli(unsigned const dest,
	unsigned const src,
	uint8_t const shamt)
{
	assert(src < 32 && dest < 32);
	return
		bits(shamt, 4, 0) << 20 |
		src << 15 |
		dest << 7 |
		MATCH_SRLI;
}

static inline uint32_t
__attribute__((const))
fence(void)
{
	return MATCH_FENCE;
}

static inline uint32_t
__attribute__((const))
auipc(unsigned dest)
{
	assert(dest < 32);
	return MATCH_AUIPC | (dest << 7);
}

#endif  /* TARGET_RISCV_OPCODES_H_ */
