#ifndef TARGET_RISCV_OPCODES_H_
#define TARGET_RISCV_OPCODES_H_

#include "encoding.h"
#include <stdint.h>
#include <assert.h>

static inline uint32_t
bits(uint32_t const value,
	unsigned const hi,
	unsigned const lo)
{
	return (value >> lo) & ((1 << (hi + 1 - lo)) - 1);
}

static inline uint32_t
bit(uint32_t const value,
	const unsigned b)
{
	return 1 & (value >> b);
}

/**
	@bug imm is int32_t
*/
static inline uint32_t
jal(unsigned const rd,
	uint32_t const imm)
{
	assert(rd < 32);
	return
		bit(imm, 20) << 31 |
		bits(imm, 10, 1) << 21 |
		bit(imm, 11) << 20 |
		bits(imm, 19, 12) << 12 |
		rd << 7 |
		MATCH_JAL;
}

static inline uint32_t
csrsi(unsigned const csr,
	uint16_t const imm)
{
	return
		csr << 20 |
		bits(imm, 4, 0) << 15 |
		MATCH_CSRRSI;
}

static inline uint32_t
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
sd(unsigned const src,
	unsigned const base,
	uint16_t const offset)
{
	assert(src < 32 && base < 32);
	return
		bits(offset, 11, 5) << 25 |
		src << 20 |
		base << 15 |
		bits(offset, 4, 0) << 7 |
		MATCH_SD;
}

static inline uint32_t
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
ld(unsigned const rd,
	unsigned const base,
	uint16_t const offset)
{
	assert(rd < 32 && base < 32);
	return
		bits(offset, 11, 0) << 20 |
		base << 15 |
		bits(rd, 4, 0) << 7 |
		MATCH_LD;
}

/**
@bug @c offset should be signed
*/
static inline uint32_t
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
csrw(unsigned const source,
	unsigned const csr)
{
	assert(source < 32);
	return
		csr << 20 |
		source << 15 |
		MATCH_CSRRW;
}

static inline uint32_t
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

/**
@bug @c csr is 12 bits only
*/
static inline uint32_t
csrr(unsigned const rd,
	unsigned const csr)
{
	assert(rd < 32);
	return
		csr << 20 |
		rd << 7 |
		MATCH_CSRRS;
}

static inline uint32_t
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
fsw(unsigned const src,
	unsigned const base,
	uint16_t const offset)
{
	assert(src < 32 && base < 32);
	return
		bits(offset, 11, 5) << 25 |
		bits(src, 4, 0) << 20 |
		base << 15 |
		bits(offset, 4, 0) << 7 |
		MATCH_FSW;
}

static inline uint32_t
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
flw(unsigned const dest,
	unsigned const base,
	uint16_t const offset)
{
	assert(dest < 32 && base < 32);
	return
		bits(offset, 11, 0) << 20 |
		base << 15 |
		bits(dest, 4, 0) << 7 |
		MATCH_FLW;
}

static inline uint32_t
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
ebreak(void)
{
	return MATCH_EBREAK;
}

static inline uint16_t
ebreak_c(void)
{
	return MATCH_C_EBREAK;
}

static inline uint32_t
wfi(void)
{
	return MATCH_WFI;
}

static inline uint32_t
fence_i(void)
{
	return MATCH_FENCE_I;
}

/**
@bug @c imm should be signed
*/
static inline uint32_t
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
fence(void)
{
	return MATCH_FENCE;
}

static inline uint32_t
auipc(unsigned dest)
{
	assert(dest < 32);
	return MATCH_AUIPC | (dest << 7);
}

#endif  /* TARGET_RISCV_OPCODES_H_ */
