#ifndef TARGET_RISCV_RISCV_H_
#define TARGET_RISCV_RISCV_H_

#include "encoding.h"

#include "target/target.h"
#include "helper/log.h"

/** @name Bit fields access macros
*/
/**@{*/
#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))
/**@}*/

/** Static array dimensions macro */
#define DIM(x) (sizeof (x) / sizeof (x)[0])

/* The register cache is statically allocated. */
#define RISCV_MAX_HARTS		(32)
#define RISCV_MAX_REGISTERS	(5000)
#define RISCV_MAX_TRIGGERS	(32)
#define RISCV_MAX_HWBPS		(16)

/** Definitions shared by code supporting all RISC-V versions. */
/**@{*/
typedef uint64_t riscv_reg_t;
typedef uint32_t riscv_insn_t;
typedef uint64_t riscv_addr_t;
/**@}*/

/** gdb's register list is defined in riscv_gdb_reg_names gdb/riscv-tdep.c in
* its source tree. We must interpret the numbers the same here. */
enum gdb_regno {
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

struct HART_register_s {
#if 0
	uint64_t saved;
#endif
	bool valid;
};

struct HART_s {
	/* Enough space to store all the registers we might need to save. */
	/**
	@todo FIXME: This should probably be a bunch of register caches.
	*/
	struct HART_register_s registers[RISCV_MAX_REGISTERS];
	/* It's possible that each core has a different supported ISA set. */
	int xlen;
	riscv_reg_t misa;

	/* The number of triggers per hart. */
	unsigned trigger_count;

	/* The number of entries in the debug buffer. */
	int debug_buffer_size;
};

struct riscv_info_s {
	unsigned dtm_version;

	struct command_context *cmd_ctx;
	void *version_specific;

	/* The number of harts on this system. */
	int hart_count;

	/* The hart that the RTOS thinks is currently being debugged. */
	int rtos_hartid;

	/* The hart that is currently being debugged.  Note that this is
	 * different than the hartid that the RTOS is expected to use.  This
	 * one will change all the time, it's more of a global argument to
	 * every function than an actual */
	int current_hartid;

	/** OpenOCD's register cache points into here.

	This is not per-hart because we just invalidate
	the entire cache when we change which hart is selected.

	@bug Use target cache instead 
	*/
	uint64_t reg_cache_values[RISCV_MAX_REGISTERS];

	/* Single buffer that contains all register names, instead of calling
	malloc for each register. Needs to be freed when reg_list is freed.

	@bug Use target cache instead
	*/
	char *reg_names;

	/**
	@bug Bad design - non-local hart information! Problem with JRC!
	*/
	struct HART_s harts[RISCV_MAX_HARTS];

	/** For each physical trigger, contains -1 if the hwbp is available, or the
	unique_id of the breakpoint/watchpoint that is using it.

	@note Note that in RTOS mode the triggers are the same across all harts the
	target controls, while otherwise only a single hart is controlled.
	*/
	int trigger_unique_id[RISCV_MAX_HWBPS];

	/* This avoids invalidating the register cache too often. */
	bool registers_initialized;

	/** This hart contains an implicit ebreak at the end of the program buffer. */
	bool impebreak;

	bool triggers_enumerated;
};
typedef struct riscv_info_s riscv_info_t;

/** Everything needs the RISC-V specific info structure, so here's a nice macro that provides that. */
static inline riscv_info_t *
__attribute__((warn_unused_result, pure))
riscv_info(struct target const *const target)
{
	assert(target);
	return target->arch_info;
}

/** RISC-V Interface */
/**@{*/

/** Steps the hart that's currently selected in the RTOS, or if there is no RTOS
* then the only hart. */
int
riscv_step_rtos_hart(struct target *target);

/**	Sets the current hart,
	which is the hart that will actually be used when issuing debug commands.
*/
/**@{*/
static inline int
__attribute__((warn_unused_result, pure))
riscv_current_hartid(struct target const *const target)
{
	riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);
	return rvi->current_hartid;
}
/**@}*/

/** @returns XLEN for the given (or current) hart. */
static inline int
__attribute__((pure))
riscv_xlen_of_hart(struct target const *const target,
	int const hartid)
{
	riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi && 0 <= hartid && hartid < RISCV_MAX_HARTS);
	assert(rvi->harts[hartid].xlen != -1);
	return rvi->harts[hartid].xlen;
}

/** @returns XLEN for current hart. */
static inline int
riscv_xlen(struct target const *const target)
{
	return riscv_xlen_of_hart(target, riscv_current_hartid(target));
}

/** Support functions for the RISC-V 'RTOS', which provides multihart support
 * without requiring multiple targets.  */

static inline void
riscv_set_rtos_hartid(struct target *const target,
	int const hartid)
{
	LOG_DEBUG("%s: setting RTOS hartid %d",
		target_name(target), hartid);
	riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);
	rvi->rtos_hartid = hartid;
}

/**@}*/

/** Lists the number of harts in the system, which are assumed to be
 * consecutive and start with `mhartid=0`. */
int
__attribute__((pure))
riscv_count_harts(struct target const *target);

/* @returns the value of the given register on the given hart.  32-bit registers
 * are zero extended to 64 bits.  */
 /**@{*/
int
riscv_set_register(struct target *target, enum gdb_regno i, riscv_reg_t v);

int
riscv_get_register(struct target *target, riscv_reg_t *value, enum gdb_regno r);

int
riscv_get_register_on_hart(struct target *target, riscv_reg_t *value, int hartid, enum gdb_regno regid);
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
	riscv_insn_t const data);

riscv_insn_t
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
