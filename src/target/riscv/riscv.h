#ifndef TARGET_RISCV_RISCV_H_
#define TARGET_RISCV_RISCV_H_

#include "encoding.h"

#include "target/target.h"
#include "helper/log.h"

/** @name Bit fields access macros
*/
/**@{*/
#define FIELD_GET(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define FIELD_SET(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))
/**@}*/

/* The register cache is statically allocated. */
#define RISCV_MAX_HARTS		(32)
#define RISCV_MAX_REGISTERS	(5000)
#define RISCV_MAX_HWBPS		(16)

/** Definitions shared by code supporting all RISC-V versions. */
/**@{*/
typedef uint64_t riscv_reg_t;
typedef uint64_t riscv_addr_t;
/**@}*/

/** gdb's register list is defined in riscv_gdb_reg_names gdb/riscv-tdep.c in
	its source tree. We must interpret the numbers the same here.
*/
enum gdb_riscv_regno {
	GDB_REGNO_ZERO = 0,        /* Read-only register, always 0.  */
	GDB_REGNO_RA = 1,          /* Return Address.  */
	GDB_REGNO_SP = 2,          /* Stack Pointer.  */
	GDB_REGNO_GP = 3,          /* Global Pointer.  */
	GDB_REGNO_TP = 4,          /* Thread Pointer.  */
	GDB_REGNO_T0,
	GDB_REGNO_T1,
	GDB_REGNO_T2,
	GDB_REGNO_S0 = 8,
	GDB_REGNO_FP = 8,          /* Frame Pointer.  */
	GDB_REGNO_S1,
	GDB_REGNO_A0 = 10,         /* First argument.  */
	GDB_REGNO_A1 = 11,         /* Second argument.  */
	GDB_REGNO_A2,
	GDB_REGNO_A3,
	GDB_REGNO_A4,
	GDB_REGNO_A5,
	GDB_REGNO_A6,
	GDB_REGNO_A7,
	GDB_REGNO_S2,
	GDB_REGNO_S3,
	GDB_REGNO_S4,
	GDB_REGNO_S5,
	GDB_REGNO_S6,
	GDB_REGNO_S7,
	GDB_REGNO_S8,
	GDB_REGNO_S9,
	GDB_REGNO_S10,
	GDB_REGNO_S11,
	GDB_REGNO_T3,
	GDB_REGNO_T4,
	GDB_REGNO_T5,
	GDB_REGNO_T6,
	GDB_REGNO_XPR31 = GDB_REGNO_T6,

	GDB_REGNO_PC = 32,
	GDB_REGNO_FPR0 = 33,
	GDB_REGNO_FT0 = GDB_REGNO_FPR0,
	GDB_REGNO_FT1,
	GDB_REGNO_FT2,
	GDB_REGNO_FT3,
	GDB_REGNO_FT4,
	GDB_REGNO_FT5,
	GDB_REGNO_FT6,
	GDB_REGNO_FT7,
	GDB_REGNO_FS0,
	GDB_REGNO_FS1,
	GDB_REGNO_FA0,
	GDB_REGNO_FA1,
	GDB_REGNO_FA2,
	GDB_REGNO_FA3,
	GDB_REGNO_FA4,
	GDB_REGNO_FA5,
	GDB_REGNO_FA6,
	GDB_REGNO_FA7,
	GDB_REGNO_FS2,
	GDB_REGNO_FS3,
	GDB_REGNO_FS4,
	GDB_REGNO_FS5,
	GDB_REGNO_FS6,
	GDB_REGNO_FS7,
	GDB_REGNO_FS8,
	GDB_REGNO_FS9,
	GDB_REGNO_FS10,
	GDB_REGNO_FS11,
	GDB_REGNO_FT8,
	GDB_REGNO_FT9,
	GDB_REGNO_FT10,
	GDB_REGNO_FT11,
	GDB_REGNO_FPR31 = GDB_REGNO_FT11,
	GDB_REGNO_CSR0 = 65,
	GDB_REGNO_TSELECT = CSR_TSELECT + GDB_REGNO_CSR0,
	GDB_REGNO_TDATA1 = CSR_TDATA1 + GDB_REGNO_CSR0,
	GDB_REGNO_TDATA2 = CSR_TDATA2 + GDB_REGNO_CSR0,
	GDB_REGNO_MISA = CSR_MISA + GDB_REGNO_CSR0,
	GDB_REGNO_DPC = CSR_DPC + GDB_REGNO_CSR0,
	GDB_REGNO_DCSR = CSR_DCSR + GDB_REGNO_CSR0,
	GDB_REGNO_DSCRATCH = CSR_DSCRATCH + GDB_REGNO_CSR0,
	GDB_REGNO_MSTATUS = CSR_MSTATUS + GDB_REGNO_CSR0,
	GDB_REGNO_CSR4095 = GDB_REGNO_CSR0 + 4095,
	GDB_REGNO_PRIV = 4161,
	GDB_REGNO_COUNT
};

/** RISC-V Interface */
/**@{*/

/** Steps the hart that's currently selected in the RTOS, or if there is no RTOS
* then the only hart. */
int
riscv_step_rtos_hart(struct target *target);

/**	Sets the current hart,
	which is the hart that will actually be used when issuing debug commands.
*/
int
__attribute__((warn_unused_result, pure))
riscv_current_hartid(struct target const *const target);

/** @returns XLEN for the given (or current) hart. */
int
__attribute__((pure))
riscv_xlen_of_hart(struct target const *const target,
	int const hartid);

/** @returns XLEN for current hart. */
static inline int
riscv_xlen(struct target const *const target)
{
	return riscv_xlen_of_hart(target, riscv_current_hartid(target));
}

/** Support functions for the RISC-V 'RTOS',
	which provides multi-hart support without requiring multiple targets.
*/
void
riscv_set_rtos_hartid(struct target *const target,
	int const hartid);

/** When using the RTOS to debug, this selects the hart that is currently being debugged.

This doesn't propagate to the hardware.
*/
void
riscv_set_all_rtos_harts(struct target const *const target);

bool
__attribute__((pure))
riscv_is_impebreak(struct target const *const target);

size_t
__attribute__((pure))
riscv_debug_buffer_size(struct target *const target);

/**@}*/

/** Lists the number of harts in the system,
	which are assumed to be consecutive and start with `mhartid=0`.

	@bug Bad assumption
*/
int
__attribute__((pure))
riscv_count_harts(struct target const *target);

/* @returns the value of the given register on the given hart.  32-bit registers
 * are zero extended to 64 bits.  */
 /**@{*/
int
riscv_set_register(struct target *target,
	enum gdb_riscv_regno i,
	riscv_reg_t v);

int
riscv_get_register(struct target *target,
	riscv_reg_t *value,
	enum gdb_riscv_regno r);

int
riscv_get_register_on_hart(struct target *target,
	riscv_reg_t *value,
	int hartid,
	enum gdb_riscv_regno regid);
/**@}*/

void
riscv_semihosting_init(struct target *target);

int
riscv_semihosting(struct target *target,
	int *p_error_code);
/**@}*/

int
riscv_write_debug_buffer(struct target *const target,
	unsigned const index,
	uint32_t const data);

uint32_t
riscv_read_debug_buffer(struct target *const target,
	unsigned const index);

int
riscv_execute_debug_buffer(struct target *const target);

int
riscv_dmi_write_u64_bits(struct target *const target);

void
riscv_fill_dmi_write_u64(struct target *const target,
	uint8_t *const buf/**<[out]*/,
	int const a,
	uint64_t const d);

void
riscv_fill_dmi_read_u64(struct target *const target,
	uint8_t *const buf/**<[out]*/,
	int a);

void
riscv_fill_dmi_nop_u64(struct target *const target,
	uint8_t *const buf/**<[out]*/);

#endif  /* TARGET_RISCV_RISCV_H_ */
