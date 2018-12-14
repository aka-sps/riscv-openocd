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
#define RISCV_MAX_HARTS 32
#define RISCV_MAX_REGISTERS 5000
#define RISCV_MAX_TRIGGERS 32
#define RISCV_MAX_HWBPS 16

#define DEFAULT_COMMAND_TIMEOUT_SEC		2
#define DEFAULT_RESET_TIMEOUT_SEC		30

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

char const *
gdb_regno_name(enum gdb_regno regno);

enum riscv_halt_reason {
	RISCV_HALT_INTERRUPT,
	RISCV_HALT_BREAKPOINT,
	RISCV_HALT_SINGLESTEP,
	RISCV_HALT_TRIGGER,
	RISCV_HALT_UNKNOWN,
	RISCV_HALT_ERROR
};

struct riscv_reg_info_s {
	struct target *target;
	unsigned custom_number;
};
typedef struct riscv_reg_info_s riscv_reg_info_t;

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

struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read;
	bool write;
	bool execute;
	int unique_id;
};

struct riscv_info_t {
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

	/** Helper functions that target the various RISC-V debug spec implementations. */
	int (__attribute__((warn_unused_result)) *get_register)(struct target *target, riscv_reg_t *value, int hid, int rid);
	int (__attribute__((warn_unused_result)) *set_register)(struct target *, int hartid, int regid, uint64_t value);
	int (__attribute__((warn_unused_result)) *select_current_hart)(struct target *);

	/**
	@todo check error code
	*/
	bool (*is_halted)(struct target *target);

	int (__attribute__((warn_unused_result)) *halt_current_hart)(struct target *);
	int (__attribute__((warn_unused_result)) *resume_current_hart)(struct target *target);
	int (__attribute__((warn_unused_result)) *step_current_hart)(struct target *target);

	/**
	@todo check error code
	*/
	int (*on_halt)(struct target *target);

	int (__attribute__((warn_unused_result)) *on_resume)(struct target *target);
	/**
	@todo check error code
	*/
	/**@{*/
	int (*on_step)(struct target *target);
	enum riscv_halt_reason (*halt_reason)(struct target *target);
	int(*write_debug_buffer)(struct target *target, unsigned index, riscv_insn_t d);
	riscv_insn_t (*read_debug_buffer)(struct target *target, unsigned index);
	/**@}*/

	int (__attribute__((warn_unused_result)) *execute_debug_buffer)(struct target *target);
	int (__attribute__((warn_unused_result)) *dmi_write_u64_bits)(struct target *target);
	/**
	@todo check error code
	*/
	/**@{*/
	void (*fill_dmi_write_u64)(struct target *target, uint8_t *buf, int a, uint64_t d);
	void (*fill_dmi_read_u64)(struct target *target, uint8_t *buf, int a);
	void (*fill_dmi_nop_u64)(struct target *target, uint8_t *buf);
	/**@}*/

	int (__attribute__((warn_unused_result)) *authdata_read)(struct target *target, uint32_t *value);
	int (__attribute__((warn_unused_result)) *authdata_write)(struct target *target, uint32_t value);

	int (__attribute__((warn_unused_result)) *dmi_read)(struct target *target, uint32_t *value, uint32_t address);
	int (__attribute__((warn_unused_result)) *dmi_write)(struct target *target, uint32_t address, uint32_t value);
	int (__attribute__((warn_unused_result)) *test_sba_config_reg)(struct target *target,
		target_addr_t legal_address,
		uint32_t num_words,
		target_addr_t illegal_address,
		bool run_sbbusyerror_test);

	int (__attribute__((warn_unused_result)) *test_compliance)(struct target *target);
};

/**
	@todo Move to separate version-related files
*/
extern struct target_type const riscv_013_target;

/**
	@bug Different targets can use different options
*/
/**@{*/
/**	Wall-clock timeout for a command/access.

	Settable via RISC-V Target commands.
*/
extern int riscv_command_timeout_sec;

/**	Wall-clock timeout after reset.

	Settable via RISC-V Target commands.
*/
extern int riscv_reset_timeout_sec;

extern bool riscv_prefer_sba;
/**@}*/

/** @name RISC-V TAP operations */
/**@{*/
void
select_dmi(struct jtag_tap *const tap);

int
__attribute__((warn_unused_result))
idcode_scan(struct jtag_tap *const tap,
	uint32_t p_in_value[1]);

int
__attribute__((warn_unused_result))
dtmcontrol_scan(struct jtag_tap *const tap,
	uint32_t const out_value,
	uint32_t *const p_in_value);
/**@}*/

/** Everything needs the RISC-V specific info structure, so here's a nice macro that provides that. */
static inline struct riscv_info_t *
__attribute__((warn_unused_result, pure))
riscv_info(struct target const *const target)
{
	assert(target);
	return target->arch_info;
}

/** OpenOCD Interface */
/** @{*/
int
riscv_openocd_poll(struct target *target);

int
riscv_openocd_halt(struct target *target);

int
riscv_openocd_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution);

int
riscv_openocd_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints);

/**@}*/

/** RISC-V Interface */
/**@{*/

/** Run control, possibly for multiple harts.  The _all_harts versions resume
 * all the enabled harts, which when running in RTOS mode is all the harts on
 * the system. */
 /**@{*/
int
riscv_halt_all_harts(struct target *target);

int
riscv_resume_all_harts(struct target *target);
/**@}*/

/** Steps the hart that's currently selected in the RTOS, or if there is no RTOS
* then the only hart. */
int
riscv_step_rtos_hart(struct target *target);

/**
	@bug Target already associated with hart with hartid
*/
static inline bool
__attribute__((pure))
riscv_supports_extension(struct target *const target,
	int const hartid,
	char const letter)
{
	unsigned num;

	if (letter >= 'a' && letter <= 'z')
		num = letter - 'a';
	else if (letter >= 'A' && letter <= 'Z')
		num = letter - 'A';
	else
		return false;

	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi && 0 <= hartid && hartid < RISCV_MAX_HARTS && num <= ('Z' - 'A'));
	return rvi->harts[hartid].misa & (1 << num);
}

static inline bool
__attribute__((pure))
riscv_rtos_enabled(struct target const *const target)
{
	assert(target);
	return !!target->rtos;
}

/** Sets the current hart, which is the hart that will actually be used when
 * issuing debug commands. */
 /**@{*/
int riscv_set_current_hartid(struct target *target, int hartid);

static inline int
__attribute__((warn_unused_result, pure))
riscv_current_hartid(struct target const *const target)
{
	struct riscv_info_t const *const rvi = riscv_info(target);
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
	struct riscv_info_t const *const rvi = riscv_info(target);
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

/** When using the RTOS to debug, this selects the hart that is currently being
 * debugged.  This doesn't propogate to the hardware. */
 /**@{*/
static inline void
riscv_set_all_rtos_harts(struct target *const target)
{
	struct riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);
	rvi->rtos_hartid = -1;
}

static inline void
riscv_set_rtos_hartid(struct target *const target,
	int const hartid)
{
	LOG_DEBUG("%s: setting RTOS hartid %d",
		target_name(target), hartid);
	struct riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);
	rvi->rtos_hartid = hartid;
}

/**@}*/

/** Lists the number of harts in the system, which are assumed to be
 * consecutive and start with `mhartid=0`. */
int
__attribute__((pure))
riscv_count_harts(struct target const *target);

/** @returns TRUE if the target has the given register on the given hart.

    @warning Always return true
*/
static inline bool
__attribute__((const))
riscv_has_register(struct target *const target,
	int const hartid,
	int const regid)
{
	(void)(target);
	(void)(hartid);
	(void)(regid);
	return true;
}

/* @returns the value of the given register on the given hart.  32-bit registers
 * are zero extended to 64 bits.  */
 /**@{*/
int
riscv_set_register(struct target *target, enum gdb_regno i, riscv_reg_t v);

int
riscv_set_register_on_hart(struct target *target, int hid, enum gdb_regno rid, uint64_t v);

int
riscv_get_register(struct target *target, riscv_reg_t *value, enum gdb_regno r);

int
riscv_get_register_on_hart(struct target *target, riscv_reg_t *value, int hartid, enum gdb_regno regid);
/**@}*/

/** Checks the state of the current hart -- "is_halted" checks the actual
 * on-device register. */
bool riscv_is_halted(struct target *target);

enum riscv_halt_reason riscv_halt_reason(struct target *target, int hartid);

/** Invalidates the register cache. */
void riscv_invalidate_register_cache(struct target *target);

/** @returns TRUE when a hart is enabled in this target. */
bool riscv_hart_enabled(struct target *target, int hartid);

int riscv_enumerate_triggers(struct target *target);

int riscv_init_registers(struct target *target);

void riscv_semihosting_init(struct target *target);
int riscv_semihosting(struct target *target, int *p_error_code);
/**@}*/

#endif  /* TARGET_RISCV_RISCV_H_ */
