/** @file
	Support for RISC-V, debug version 0.13, which is currently (2/4/17) the
	latest draft.
*/

#include "riscv.h"
#include "opcodes.h"
#include "debug_defines.h"
#include "program.h"
#include "batch.h"

#include "target/algorithm.h"
#include "target/target_type.h"
#include "jtag/jtag.h"
#include "target/register.h"
#include "target/breakpoints.h"
#include "rtos/rtos.h"
#include "rtos/riscv_debug.h"

/** @file
 Since almost everything can be accomplish by scanning the dbus register, all
 functions here assume dbus is already selected. The exception are functions
 called directly by OpenOCD, which can't assume anything about what's
 currently in IR. They should set IR to dbus explicitly.

 @bug Not robust strategy
 */

/** @file
	Code structure
	
	At the bottom of the stack are the OpenOCD JTAG functions:
		- jtag_add_[id]r_scan
		- jtag_execute_query
		- jtag_add_runtest
	
	There are a few functions to just instantly shift a register and get its
	value:
		- dtmcontrol_scan
		- idcode_scan
		- dbus_scan
	
	Because doing one scan and waiting for the result is slow, most functions
	batch up a bunch of dbus writes and then execute them all at once. They use
	the scans "class" for this:
		- scans_new
		- scans_delete
		- scans_execute
		- scans_add_...

	Usually you new(), call a bunch of add functions, then execute() and look
	at the results by calling scans_get...()
	
	Optimized functions will directly use the scans class above, but slightly
	lazier code will use the cache functions that in turn use the scans
	functions:
		- cache_get...
		- cache_set...
		- cache_write

	cache_set... update a local structure, which is then synced to the target
	with cache_write(). Only Debug RAM words that are actually changed are sent
	to the target. Afterwards use cache_get... to read results.
*/

#define DMI_DATA1		(DMI_DATA0 + 1)
#define DMI_PROGBUF1	(DMI_PROGBUF0 + 1)

#define CSR_DCSR_CAUSE_SWBP		(1)
#define CSR_DCSR_CAUSE_TRIGGER	(2)
#define CSR_DCSR_CAUSE_DEBUGINT	(3)
#define CSR_DCSR_CAUSE_STEP		(4)
#define CSR_DCSR_CAUSE_HALT		(5)

#define RISCV_MAX_TRIGGERS		(32)

/** Static array dimensions macro */
#define DIM(x) (sizeof (x) / sizeof (x)[0])

#define COMPLIANCE_TEST(b, message) \
{                                   \
	bool const pass = !!(b);        \
	if (pass) {                     \
		++passed_tests;             \
	}                               \
	LOG_INFO("%s test %d (%s)", pass ? "PASSED" : "FAILED",  ++total_tests, message); \
}

#define COMPLIANCE_MUST_PASS(b) COMPLIANCE_TEST(ERROR_OK == (b), "Regular calls must return ERROR_OK")

#define COMPLIANCE_READ(target, addr, value) COMPLIANCE_MUST_PASS(dmi_read(target, addr, value))
#define COMPLIANCE_WRITE(target, addr, value) COMPLIANCE_MUST_PASS(dmi_write(target, addr, value))

#define COMPLIANCE_CHECK_RO(target, addr)                               \
{                                                                       \
	uint32_t orig;                                                      \
	COMPLIANCE_READ(target, &orig, addr);                               \
	COMPLIANCE_WRITE(target, addr, ~orig);                              \
	uint32_t inverse;                                                   \
	COMPLIANCE_READ(target, &inverse, addr);                            \
	COMPLIANCE_TEST(orig == inverse, "Register must be read-only");     \
}

#define DUMP_FIELD_BUFFER_SIZE (500)

/** @name JTAG registers. */
/**@{*/
#define DTMCONTROL_VERSION			(0xF)
/**@}*/

#define DEFAULT_COMMAND_TIMEOUT_SEC		(2)
#define DEFAULT_RESET_TIMEOUT_SEC		(30)

/** @name External handlers

	@todo place in header
*/
/**@{*/
__COMMAND_HANDLER(handle_common_semihosting_command);
__COMMAND_HANDLER(handle_common_semihosting_fileio_command);
__COMMAND_HANDLER(handle_common_semihosting_resumable_exit_command);
__COMMAND_HANDLER(handle_common_semihosting_cmdline);
/**@}*/

enum dmi_op_e {
	DMI_OP_NOP = 0,
	DMI_OP_READ = 1,
	DMI_OP_WRITE = 2
};
typedef enum dmi_op_e dmi_op_t;

enum dmi_status_e {
	DMI_STATUS_SUCCESS = 0,
	DMI_STATUS_FAILED = 2,
	DMI_STATUS_BUSY = 3
};
typedef enum dmi_status_e dmi_status_t;

/** Debug Bus registers. */
enum CMDERR_e {
	CMDERR_NONE = 0,
	CMDERR_BUSY = 1,
	CMDERR_NOT_SUPPORTED = 2,
	CMDERR_EXCEPTION = 3,
	CMDERR_HALT_RESUME = 4,
	CMDERR_OTHER = 7,
};

enum yes_no_maybe_e {
	YNM_MAYBE,
	YNM_YES,
	YNM_NO
};
typedef enum yes_no_maybe_e yes_no_maybe_t;

enum memory_space_e {
	SPACE_DMI_DATA,
	SPACE_DMI_PROGBUF,
	SPACE_DMI_RAM
};
typedef enum memory_space_e memory_space_t;

enum riscv_halt_reason_e {
	RISCV_HALT_INTERRUPT,
	RISCV_HALT_BREAKPOINT,
	RISCV_HALT_SINGLESTEP,
	RISCV_HALT_TRIGGER,
	RISCV_HALT_UNKNOWN,
	RISCV_HALT_ERROR
};
typedef enum riscv_halt_reason_e riscv_halt_reason_t;

struct dm013_info_s {
	struct list_head list;
	int abs_chain_position;

	/* Indicates we already reset this DM, so don't need to do it again. */
	bool was_reset;

	/* Targets that are connected to this DM. */
	struct list_head target_list;

	/* The currently selected hartid on this DM. */
	int current_hartid;
};
typedef struct dm013_info_s dm013_info_t;

struct target_list_s {
	struct list_head list;
	struct target *target;
};
typedef struct target_list_s target_list_t;

struct riscv_013_info_s {
	/** Number of address bits in the dbus register. */
	unsigned abits;

	/** Number of abstract command data registers. */
	unsigned datacount;

	/** Number of words in the Program Buffer. */
	unsigned progbufsize;

	/** We cache the read-only bits of sbcs here. */
	uint32_t sbcs;

	yes_no_maybe_t progbuf_writable;

	/** We only need the address so that we know the alignment of the buffer. */
	riscv_addr_t progbuf_address;

	/** Number of run-test/idle cycles the target requests we do after each dbus
	* access. */
	unsigned dtmcontrol_idle;

	/** This value is incremented every time a dbus access comes back as "busy".
	* It's used to determine how many run-test/idle cycles to feed the target
	* in between accesses. */
	unsigned dmi_busy_delay;

	/** Number of run-test/idle cycles to add between consecutive bus master
	* reads/writes respectively. */
	unsigned bus_master_write_delay;
	unsigned bus_master_read_delay;

	/** This value is increased every time we tried to execute two commands
	* consecutively, and the second one failed because the previous hadn't
	* completed yet.  It's used to add extra run-test/idle cycles after
	* starting a command, so we don't have to waste time checking for busy to
	* go low. */
	unsigned ac_busy_delay;

	bool abstract_read_csr_supported;
	bool abstract_write_csr_supported;
	bool abstract_read_fpr_supported;
	bool abstract_write_fpr_supported;

	/**	When a function returns some error due to a failure indicated by the target in cmderr,
	the caller can look here to see what that error was.
	(Compare with errno.) */
	uint8_t cmderr;

	/** @name Some fields from hartinfo. */
	/**@{*/
	uint8_t datasize;
	uint8_t dataaccess;
	int16_t dataaddr;
	/**@}*/

	/** The width of the hartsel field. */
	unsigned hartsellen;

	/** DM that provides access to this target. */
	dm013_info_t *dm;
};
typedef struct riscv_013_info_s riscv_013_info_t;

/**
@todo Possible duplicates struct target fields
*/
struct scratch_mem_s {
	/**
	@todo How can the debugger access this memory?
	*/
	memory_space_t memory_space;

	/** Memory address to access the scratch memory from the hart. */
	riscv_addr_t hart_address;

	/** Memory address to access the scratch memory from the debugger. */
	riscv_addr_t debug_address;

	struct working_area *area;
};
typedef struct scratch_mem_s scratch_mem_t;

struct descr {
	unsigned address;
	uint64_t mask;
	char const *name;
};

struct range_s {
	uint16_t low;
	uint16_t high;
};
typedef struct range_s range_t;

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

struct riscv_reg_info_s {
	struct target *target;
	unsigned custom_number;
};
typedef struct riscv_reg_info_s riscv_reg_info_t;

struct csr_info {
	unsigned number;
	char const *name;
};

static struct csr_info csr_info[] = {
#define DECLARE_CSR(name, number) { number, #name },
#include "encoding.h"
#undef DECLARE_CSR
};

static struct descr const description[] = {
	{DMI_DMCONTROL, DMI_DMCONTROL_HALTREQ, "haltreq"},
	{DMI_DMCONTROL, DMI_DMCONTROL_RESUMEREQ, "resumereq"},
	{DMI_DMCONTROL, DMI_DMCONTROL_HARTRESET, "hartreset"},
	{DMI_DMCONTROL, DMI_DMCONTROL_HASEL, "hasel"},
	{DMI_DMCONTROL, DMI_DMCONTROL_HARTSELHI, "hartselhi"},
	{DMI_DMCONTROL, DMI_DMCONTROL_HARTSELLO, "hartsello"},
	{DMI_DMCONTROL, DMI_DMCONTROL_NDMRESET, "ndmreset"},
	{DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE, "dmactive"},
	{DMI_DMCONTROL, DMI_DMCONTROL_ACKHAVERESET, "ackhavereset"},

	{DMI_DMSTATUS, DMI_DMSTATUS_IMPEBREAK, "impebreak"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ALLHAVERESET, "allhavereset"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ANYHAVERESET, "anyhavereset"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ALLRESUMEACK, "allresumeack"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ANYRESUMEACK, "anyresumeack"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ALLNONEXISTENT, "allnonexistent"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ANYNONEXISTENT, "anynonexistent"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ALLUNAVAIL, "allunavail"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ANYUNAVAIL, "anyunavail"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ALLRUNNING, "allrunning"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ANYRUNNING, "anyrunning"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ALLHALTED, "allhalted"},
	{DMI_DMSTATUS, DMI_DMSTATUS_ANYHALTED, "anyhalted"},
	{DMI_DMSTATUS, DMI_DMSTATUS_AUTHENTICATED, "authenticated"},
	{DMI_DMSTATUS, DMI_DMSTATUS_AUTHBUSY, "authbusy"},
	{DMI_DMSTATUS, DMI_DMSTATUS_DEVTREEVALID, "devtreevalid"},
	{DMI_DMSTATUS, DMI_DMSTATUS_VERSION, "version"},

	{DMI_ABSTRACTCS, DMI_ABSTRACTCS_PROGBUFSIZE, "progbufsize"},
	{DMI_ABSTRACTCS, DMI_ABSTRACTCS_BUSY, "busy"},
	{DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR, "cmderr"},
	{DMI_ABSTRACTCS, DMI_ABSTRACTCS_DATACOUNT, "datacount"},

	{DMI_COMMAND, DMI_COMMAND_CMDTYPE, "cmdtype"},

	{DMI_SBCS, DMI_SBCS_SBREADONADDR, "sbreadonaddr"},
	{DMI_SBCS, DMI_SBCS_SBACCESS, "sbaccess"},
	{DMI_SBCS, DMI_SBCS_SBAUTOINCREMENT, "sbautoincrement"},
	{DMI_SBCS, DMI_SBCS_SBREADONDATA, "sbreadondata"},
	{DMI_SBCS, DMI_SBCS_SBERROR, "sberror"},
	{DMI_SBCS, DMI_SBCS_SBASIZE, "sbasize"},
	{DMI_SBCS, DMI_SBCS_SBACCESS128, "sbaccess128"},
	{DMI_SBCS, DMI_SBCS_SBACCESS64, "sbaccess64"},
	{DMI_SBCS, DMI_SBCS_SBACCESS32, "sbaccess32"},
	{DMI_SBCS, DMI_SBCS_SBACCESS16, "sbaccess16"},
	{DMI_SBCS, DMI_SBCS_SBACCESS8, "sbaccess8"},
};

static LIST_HEAD(dm_list);

/**	Wall-clock timeout for a command/access.

Settable via RISC-V Target commands.
*/
static int riscv_command_timeout_sec = DEFAULT_COMMAND_TIMEOUT_SEC;

/**	Wall-clock timeout after reset.

Settable via RISC-V Target commands.
*/
static int riscv_reset_timeout_sec = DEFAULT_RESET_TIMEOUT_SEC;

static bool riscv_prefer_sba = false;

/** In addition to the ones in the standard spec, we'll also expose additional CSRs in this list.

The list is either NULL, or a series of ranges (inclusive), terminated with
1,0.

@bug Different targets can use different lists
*/
range_t *expose_csr = NULL;

/** In addition to the ones in the standard spec, we'll also expose additional custom registers. */
range_t *expose_custom = NULL;

static void
decode_dmi(char buffer[DUMP_FIELD_BUFFER_SIZE]/**<[out]*/,
	unsigned const address/**<[in]*/,
	unsigned const data/**<[in]*/)
{
	char *p_text = buffer;
	*p_text = '\0';

	char *const end_of_buffer = p_text + DUMP_FIELD_BUFFER_SIZE - 1;
	*end_of_buffer = '\0';

	struct descr const *p_descr = description;
	struct descr const *p_descr_end = description + DIM(description);

	for (; p_descr != p_descr_end; ++p_descr) {
		if (p_descr->address != address)
			continue;

		uint64_t const mask = p_descr->mask;
		unsigned const value = FIELD_GET(data, mask);

		/**
		@todo Posiible is break?
		*/
		if (0 == value)
			continue;

		if (buffer != p_text)
			*p_text++ = ' ';

		if (mask & (mask >> 1)) {
			/* If the field is more than 1 bit wide. */
			snprintf(p_text, end_of_buffer - p_text, "%s=%d", p_descr->name, value);
		} else {
			strncpy(p_text, p_descr->name, end_of_buffer - p_text);
		}
		p_text = strchr(p_text, '\0');
	}
}

static void
riscv_013_dump_field(struct scan_field const *const field)
{
	static char const *const op_string[] = {"-", "r", "w", "?"};
	static char const *const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	assert(field);
	uint64_t const out_value = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned const out_op = FIELD_GET(out_value, DTM_DMI_OP);
	unsigned const out_data = FIELD_GET(out_value, DTM_DMI_DATA);
	unsigned const out_address = out_value >> DTM_DMI_ADDRESS_OFFSET;

	uint64_t const in_value = buf_get_u64(field->in_value, 0, field->num_bits);
	unsigned const in_op = FIELD_GET(in_value, DTM_DMI_OP);
	unsigned const in_data = FIELD_GET(in_value, DTM_DMI_DATA);
	unsigned const in_address = in_value >> DTM_DMI_ADDRESS_OFFSET;

	log_printf_lf(LOG_LVL_DEBUG,
		__FILE__, __LINE__, "scan",
		"%db %s %08x @%02x -> %s %08x @%02x",
		field->num_bits,
		op_string[out_op], out_data, out_address,
		status_string[in_op], in_data, in_address);

	char out_text[DUMP_FIELD_BUFFER_SIZE];
	decode_dmi(out_text, out_address, out_data);

	char in_text[DUMP_FIELD_BUFFER_SIZE];
	decode_dmi(in_text, in_address, in_data);

	if (in_text[0] || out_text[0]) {
		log_printf_lf(LOG_LVL_DEBUG, __FILE__, __LINE__, "scan", "%s -> %s",
			out_text, in_text);
	}
}

static inline riscv_013_info_t *
__attribute__((pure))
get_info(struct target const *const target)
{
	assert(target);
	riscv_info_t const *const info = target->arch_info;
	assert(info);
	return info->version_specific;
}

static dmi_status_t
__attribute__((warn_unused_result))
dmi_scan(struct target *const target,
	uint32_t *const address_in /*<[out]*/,
	uint32_t *const data_in /*<[out]*/,
	dmi_op_t const op,
	uint32_t const address_out,
	uint32_t const data_out,
	bool const exec /**<[in] If this is set,
					assume the scan results in an execution,
					so more run-test/idle cycles may be required.
					*/)
{
	riscv_013_info_t *const info = get_info(target);

	assert(info);
	assert(info->abits != 0);

	uint8_t out_buffer[8];
	buf_set_u32(out_buffer, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, op);
	buf_set_u32(out_buffer, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, data_out);
	buf_set_u32(out_buffer, DTM_DMI_ADDRESS_OFFSET, info->abits, address_out);

	uint8_t in_buffer[8] = {0};
	typedef struct scan_field scan_field_t;
	scan_field_t const field = {
		.num_bits = info->abits + DTM_DMI_OP_LENGTH + DTM_DMI_DATA_LENGTH,
		.out_value = out_buffer,
		.in_value = in_buffer
	};

	/** @pre Assumed dbus is already selected. */
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	{
		int const idle_count = info->dmi_busy_delay + (exec ? info->ac_busy_delay : 0);

		if (idle_count)
			jtag_add_runtest(idle_count, TAP_IDLE);
	}

	if (ERROR_OK != jtag_execute_queue()) {
		LOG_ERROR("%s: dmi_scan failed jtag scan", target_name(target));
		return DMI_STATUS_FAILED;
	}

	if (data_in)
		*data_in = buf_get_u32(in_buffer, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);

	if (address_in)
		*address_in = buf_get_u32(in_buffer, DTM_DMI_ADDRESS_OFFSET, info->abits);

	riscv_013_dump_field(&field);

	return buf_get_u32(in_buffer, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
}

static void
select_instruction(struct jtag_tap *const tap,
	uint8_t const instruction_buffer[1])
{
	assert(tap);
	assert(0 < tap->ir_length && tap->ir_length <= UINT8_MAX);
	typedef struct scan_field scan_field_t;
	scan_field_t const field = {
		.num_bits = tap->ir_length,
		.out_value = instruction_buffer,
	};

	jtag_add_ir_scan(tap, &field, TAP_IDLE);
}

static void
select_dmi(struct jtag_tap *const tap)
{
	static uint8_t const instruction_buffer[1] = {DTM_DMI};
	select_instruction(tap, instruction_buffer);
}

static int
__attribute__((warn_unused_result))
uint32_instruction_scan(struct jtag_tap *const tap,
	uint8_t const instruction,
	char const *const instruction_name,
	uint32_t const out_value,
	uint32_t *const p_in_value)
{
	typedef struct scan_field scan_field_t;
	uint8_t out_buffer[sizeof(uint32_t)] = {};
	uint8_t in_buffer[sizeof(uint32_t)] = {};
	scan_field_t const field = {
		.num_bits = CHAR_BIT * sizeof(uint32_t),
		.out_value = out_buffer,
		.in_value = p_in_value ? in_buffer : NULL,
	};

	select_instruction(tap, &instruction);
	buf_set_u32(out_buffer, 0, CHAR_BIT * sizeof(uint32_t), out_value);
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	/** Always return to @c dmi.

	@bug Non robust strategy
	*/
	select_dmi(tap);

	int const error_code = jtag_execute_queue();

	if (ERROR_OK != error_code) {
		LOG_ERROR("%s: jtag IR/DR scan failed: %d",
			jtag_tap_name(tap), error_code);
	} else if (p_in_value) {
		*p_in_value = buf_get_u32(field.in_value, 0, CHAR_BIT * sizeof(uint32_t));
		LOG_DEBUG("%s: %s: 0x%" PRIx32 " -> 0x%" PRIx32,
			jtag_tap_name(tap), instruction_name, out_value, *p_in_value);
	} else {
		LOG_DEBUG("%s: %s: 0x%" PRIx32,
			jtag_tap_name(tap), instruction_name, out_value);
	}

	return error_code;
}

static int
dtmcontrol_scan(struct jtag_tap *const tap,
	uint32_t const out_value,
	uint32_t *const p_in_value)
{
	return uint32_instruction_scan(tap, DTM_DTMCS, "DTMCONTROL", out_value, p_in_value);
}

static int
increase_dmi_busy_delay(struct target *const target)
{
	riscv_013_info_t *const info = get_info(target);
	assert(info);
	info->dmi_busy_delay += info->dmi_busy_delay / 10 + 1;
	LOG_DEBUG("%s: dtmcontrol_idle=%u, dmi_busy_delay=%u, ac_busy_delay=%u",
		target_name(target),
		info->dtmcontrol_idle,
		info->dmi_busy_delay,
		info->ac_busy_delay);

	return dtmcontrol_scan(target->tap, DTM_DTMCS_DMIRESET, NULL);
}

/** @return error code */
static int
__attribute__((warn_unused_result))
dmi_op_timeout(struct target *const target,
	uint32_t *const data_in /**<[out]*/,
	int const dmi_op_code,
	uint32_t const address,
	uint32_t const data_out,
	int const timeout_sec)
{
	select_dmi(target->tap);

	uint32_t address_in;

	char const *op_name;

	switch (dmi_op_code) {
	case DMI_OP_NOP:
		op_name = "nop";
		break;

	case DMI_OP_READ:
		op_name = "read";
		break;

	case DMI_OP_WRITE:
		op_name = "write";
		break;

	default:
		LOG_ERROR("%s: Invalid DMI operation: %d", target_name(target), dmi_op_code);
		return ERROR_TARGET_INVALID;
	}

	time_t const start = time(NULL);

	/* This first loop performs the request.  Note that if for some reason this
	* stays busy, it is actually due to the previous access. */
	for (;;) {
		dmi_status_t const status =
			dmi_scan(target, NULL, NULL, dmi_op_code, address, data_out, false);

		if (DMI_STATUS_SUCCESS == status)
			break;

		if (DMI_STATUS_BUSY != status) {
			LOG_ERROR("%s: failed %s at 0x%x, status=%d",
				target_name(target), op_name, address, status);
			return ERROR_TARGET_FAILURE;
		}

		increase_dmi_busy_delay(target);

		if (start + timeout_sec < time(NULL)) {
			LOG_ERROR("%s: timeout", target_name(target));
			return ERROR_TARGET_TIMEOUT;
		}
	}

	/* This second loop ensures the request succeeded, and gets back data.
	* Note that NOP can result in a 'busy' result as well, but that would be
	* noticed on the next DMI access we do. */
	for (;;) {
		dmi_status_t const status =
			dmi_scan(target, &address_in, data_in, DMI_OP_NOP, address, 0, false);

		if (DMI_STATUS_SUCCESS == status)
			break;

		if (DMI_STATUS_BUSY != status) {
			LOG_ERROR("%s: failed %s (NOP) at 0x%x, status=%d",
				target_name(target), op_name, address, status);
			return ERROR_TARGET_FAILURE;
		}

		increase_dmi_busy_delay(target);

		if (start + timeout_sec < time(NULL)) {
			LOG_ERROR("%s: timeout", target_name(target));
			return ERROR_TARGET_TIMEOUT;
		}
	}

	return ERROR_OK;
}

/** @return error code */
static int
riscv_013_register_read(struct target *const target, uint64_t *value, uint32_t number);

/** @return error code */
static int
__attribute__((warn_unused_result))
riscv_013_register_read_direct(struct target *const target, uint64_t *value, uint32_t number);

/** @return error code */
static int
__attribute__((warn_unused_result))
riscv_013_register_write_direct(struct target *const target, unsigned number, uint64_t value);

/** @return error code */
static int
riscv_013_read_memory(struct target *const target, target_addr_t address, uint32_t size, uint32_t count, uint8_t *buffer);

/** @return error code */
static int
riscv_013_write_memory(struct target *const target, target_addr_t address, uint32_t size, uint32_t count, const uint8_t *buffer);

/**
@return the DM structure for this target. If there isn't one, find it in the
global list of DMs. If it's not in there, then create one and initialize it
to 0.
*/
static dm013_info_t *
__attribute__((warn_unused_result))
get_dm(struct target *const target)
{
	riscv_013_info_t *const info = get_info(target);
	assert(info);

	if (info->dm)
		return info->dm;

	assert(target && target->tap);
	int abs_chain_position = target->tap->abs_chain_position;

	dm013_info_t *entry;
	dm013_info_t *dm = NULL;

	list_for_each_entry(entry, &dm_list, list)
	{
		if (entry->abs_chain_position == abs_chain_position) {
			dm = entry;
			break;
		}
	}

	if (!dm) {
		dm = calloc(1, sizeof(dm013_info_t));
		dm->abs_chain_position = abs_chain_position;
		dm->current_hartid = -1;
		INIT_LIST_HEAD(&dm->target_list);
		list_add(&dm->list, &dm_list);
	}

	info->dm = dm;
	target_list_t *target_entry;

	list_for_each_entry(target_entry, &dm->target_list, list)
	{
		if (target_entry->target == target)
			return dm;
	}

	target_entry = calloc(1, sizeof *target_entry);
	assert(target_entry);
	target_entry->target = target;
	list_add(&target_entry->list, &dm->target_list);

	return dm;
}

static uint32_t
__attribute__((const))
set_hartsel(uint32_t initial,
	uint32_t const index)
{
	initial &= ~DMI_DMCONTROL_HARTSELLO;
	initial &= ~DMI_DMCONTROL_HARTSELHI;

	uint32_t const index_lo = index & ((1 << DMI_DMCONTROL_HARTSELLO_LENGTH) - 1);
	initial |= index_lo << DMI_DMCONTROL_HARTSELLO_OFFSET;
	uint32_t const index_hi = index >> DMI_DMCONTROL_HARTSELLO_LENGTH;
	assert(index_hi < 1 << DMI_DMCONTROL_HARTSELHI_LENGTH);
	initial |= index_hi << DMI_DMCONTROL_HARTSELHI_OFFSET;

	return initial;
}

/** @return error code */
static int
dmi_op(struct target *const target/**<[in]*/,
	uint32_t *const data_in/**<[out]*/,
	int const dmi_oper/**<[in]*/,
	uint32_t const address/**<[in]*/,
	uint32_t const data_out/**<[in]*/)
{
	int const result =
		dmi_op_timeout(target, data_in, dmi_oper, address, data_out, riscv_command_timeout_sec);

	if (ERROR_TARGET_TIMEOUT == result) {
		LOG_ERROR("%s: DMI operation didn't complete in %d seconds."
			" The target is either really slow or broken."
			" You could increase the timeout with riscv set_command_timeout_sec.",
			target_name(target), riscv_command_timeout_sec);
	}

	return result;
}

/** @return error code */
static inline int
__attribute__((warn_unused_result))
dmi_read(struct target *const target,
	uint32_t *const value/**<[out]*/,
	uint32_t const address)
{
	return dmi_op(target, value, DMI_OP_READ, address, 0);
}

/** @return error code */
static inline int
__attribute__((warn_unused_result))
dmi_write(struct target *const target,
	uint32_t const address,
	uint32_t const value)
{
	return dmi_op(target, NULL, DMI_OP_WRITE, address, value);
}

/** @return error code */
static int
dmstatus_read_timeout(struct target *const target,
	uint32_t *const dmstatus/**<[out]*/,
	bool const authenticated,
	unsigned const timeout_sec)
{
	{
		int const error_code =
			dmi_op_timeout(target, dmstatus, DMI_OP_READ, DMI_DMSTATUS, 0, timeout_sec);

		if (ERROR_OK != error_code)
			return error_code;
	}

	if (authenticated && 0 == FIELD_GET(*dmstatus, DMI_DMSTATUS_AUTHENTICATED)) {
		assert(dmstatus);
		LOG_ERROR("%s: Debugger is not authenticated to target Debug Module (dmstatus=0x%x)."
			" Use `riscv authdata_read` and `riscv authdata_write` commands to authenticate.",
			target_name(target), *dmstatus);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

/** @return error code */
static int
dmstatus_read(struct target *const target,
	uint32_t *const dmstatus/**<[out]*/,
	bool const authenticated)
{
	return
		dmstatus_read_timeout(target,
			dmstatus,
			authenticated,
			riscv_command_timeout_sec);
}

static void
increase_ac_busy_delay(struct target *const target)
{
	riscv_013_info_t *const info = get_info(target);
	assert(info);
	info->ac_busy_delay += info->ac_busy_delay / 10 + 1;
	LOG_DEBUG("%s: dtmcontrol_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d", target_name(target),
		info->dtmcontrol_idle, info->dmi_busy_delay,
		info->ac_busy_delay);
}

/** @return error code */
static int
wait_for_idle(struct target *const target,
	uint32_t *const abstractcs)
{
	riscv_013_info_t *const info = get_info(target);
	assert(info);
	time_t const start = time(NULL);
	assert(abstractcs);

	for (;;) {
		{
			int const error_code = dmi_read(target, abstractcs, DMI_ABSTRACTCS);

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (0 == FIELD_GET(*abstractcs, DMI_ABSTRACTCS_BUSY))
			return ERROR_OK;

		if (time(NULL) > start + riscv_command_timeout_sec) {
			info->cmderr = FIELD_GET(*abstractcs, DMI_ABSTRACTCS_CMDERR);

			if (info->cmderr != CMDERR_NONE) {
				static char const *const errors[8] = {
					"none",
					"busy",
					"not supported",
					"exception",
					"halt/resume",
					"reserved",
					"reserved",
					"other"
				};

				LOG_ERROR("%s: Abstract command ended in error '%s' (abstractcs=0x%x)",
					target_name(target),
					errors[info->cmderr],
					*abstractcs);
			}

			LOG_ERROR("%s: Timed out after %ds waiting for busy to go low (abstractcs=0x%x). "
				"Increase the timeout with riscv set_command_timeout_sec.",
				target_name(target), riscv_command_timeout_sec, *abstractcs);

			return ERROR_TARGET_TIMEOUT;
		}
	}
}

/** @return error code */
static int
__attribute__((warn_unused_result))
execute_abstract_command(struct target *const target,
	uint32_t const command)
{
	LOG_DEBUG("%s: command=0x%x", target_name(target), command);
	{
		int const error_code = dmi_write(target, DMI_COMMAND, command);

		if (ERROR_OK != error_code)
			return error_code;
	}

	uint32_t abstractcs = 0;
	wait_for_idle(target, &abstractcs);

	riscv_013_info_t *const info = get_info(target);
	assert(info);
	info->cmderr = FIELD_GET(abstractcs, DMI_ABSTRACTCS_CMDERR);

	if (0 != info->cmderr) {
		LOG_DEBUG("%s: command 0x%" PRIx32 " failed; abstractcs=0x%" PRIx32 " cmderr=0x%" PRIx32,
			target_name(target), command, abstractcs, info->cmderr);
		/* Clear the error. */
		int const error = dmi_write(target, DMI_ABSTRACTCS, FIELD_SET(0, DMI_ABSTRACTCS_CMDERR, info->cmderr));
		(void)(error);
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

/**
@bug Unhandled errors
*/
static riscv_reg_t
read_abstract_arg(struct target *const target,
	unsigned const index,
	unsigned const size_bits)
{
	riscv_reg_t value = 0;
	uint32_t v;
	unsigned const offset = index * size_bits / 32;

	switch (size_bits) {
	default:
		LOG_ERROR("%s: Unsupported size: %d", target_name(target), size_bits);
		return ~0;

	case 64:
		{
			int const error_code = dmi_read(target, &v, DMI_DATA0 + offset + 1);

			if (ERROR_OK != error_code)
				LOG_WARNING("%s: unhandled dmi_read error, error code %d, invalid value return",
					target_name(target), error_code);
		}

		value |= ((uint64_t)v) << 32;
		/* falls through */

	case 32:
		{
			int const error_code = dmi_read(target, &v, DMI_DATA0 + offset);

			if (ERROR_OK != error_code)
				LOG_WARNING("%s: unhandled dmi_read error, error code %d, invalid value return",
					target_name(target), error_code);
		}
		value |= v;
	}

	return value;
}

/** @return error code */
static int
write_abstract_arg(struct target *const target,
	unsigned const index,
	riscv_reg_t const value,
	unsigned const size_bits)
{
	unsigned const offset = index * size_bits / 32;

	int error_code = ERROR_OK;

	switch (size_bits) {
	default:
		LOG_ERROR("%s: Unsupported size: %d", target_name(target), size_bits);
		return ERROR_TARGET_INVALID;
	case 64:
		error_code = dmi_write(target, DMI_DATA0 + offset + 1, value >> 32);
		/* falls through */
	case 32:
		if (ERROR_OK == error_code)
			error_code = dmi_write(target, DMI_DATA0 + offset, value);
	}

	return error_code;
}

static uint32_t
access_register_command(struct target *const target/**<[inout]*/,
	uint32_t const number/**<[in]*/,
	unsigned const size/**<[in] in bits*/,
	uint32_t const flags/**<[in]*/)
{
	uint32_t command = FIELD_SET(0, DMI_COMMAND_CMDTYPE, 0);

	switch (size) {
	case 32:
		command = FIELD_SET(command, AC_ACCESS_REGISTER_SIZE, 2);
		break;
	case 64:
		command = FIELD_SET(command, AC_ACCESS_REGISTER_SIZE, 3);
		break;
	default:
		assert(0);
	}

	if (number <= GDB_REGNO_XPR31) {
		command = FIELD_SET(command, AC_ACCESS_REGISTER_REGNO, 0x1000 + number - GDB_REGNO_ZERO);
	} else if (GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31) {
		command = FIELD_SET(command, AC_ACCESS_REGISTER_REGNO, 0x1020 + number - GDB_REGNO_FPR0);
	} else if (GDB_REGNO_CSR0 <= number && number <= GDB_REGNO_CSR4095) {
		command = FIELD_SET(command, AC_ACCESS_REGISTER_REGNO, number - GDB_REGNO_CSR0);
	} else if (GDB_REGNO_COUNT <= number) {
		/* Custom register. */
		assert(target && target->reg_cache && target->reg_cache->reg_list && number < target->reg_cache->num_regs);
		riscv_reg_info_t *const reg_info = target->reg_cache->reg_list[number].arch_info;
		assert(reg_info);
		command = FIELD_SET(command, AC_ACCESS_REGISTER_REGNO, 0xc000 + reg_info->custom_number);
	}

	return command | flags;
}

/** @return error code */
static int
register_read_abstract(struct target *const target,
	uint64_t *const value,
	uint32_t const number,
	unsigned const size)
{
	riscv_013_info_t *const info = get_info(target);
	assert(info);

	if ((GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31 && !info->abstract_read_fpr_supported) ||
		(GDB_REGNO_CSR0 <= number && number <= GDB_REGNO_CSR4095 && !info->abstract_read_csr_supported)) {
		LOG_ERROR("%s: not supported abstract register read for register number %d",
			target_name(target), number);
		return ERROR_TARGET_INVALID;
	}

	uint32_t const command = access_register_command(target, number, size, AC_ACCESS_REGISTER_TRANSFER);

	{
		int const error_code = execute_abstract_command(target, command);

		if (ERROR_OK != error_code) {
			if (CMDERR_NOT_SUPPORTED == info->cmderr) {
				if (GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31) {
					info->abstract_read_fpr_supported = false;
					LOG_INFO("%s: Disabling abstract command reads from FPRs.",
						target_name(target));
				} else if (GDB_REGNO_CSR0 <= number && number <= GDB_REGNO_CSR4095) {
					info->abstract_read_csr_supported = false;
					LOG_INFO("%s: Disabling abstract command reads from CSRs.", target_name(target));
				}
			}

			return error_code;
		}
	}

	if (value)
		*value = read_abstract_arg(target, 0, size);

	return ERROR_OK;
}

/** @return error code */
static int
register_write_abstract(struct target *const target,
	uint32_t const number,
	uint64_t const value,
	unsigned const size)
{
	riscv_013_info_t *const info = get_info(target);
	assert(info);

	if (!info->abstract_write_fpr_supported &&
		((GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31) ||
		(GDB_REGNO_CSR0 <= number && number <= GDB_REGNO_CSR4095))
		) {
		LOG_ERROR("%s:", target_name(target));
		return ERROR_TARGET_INVALID;
	}

	uint32_t const command =
		access_register_command(target, number, size,
			AC_ACCESS_REGISTER_TRANSFER | AC_ACCESS_REGISTER_WRITE);

	{
		int const error_code = write_abstract_arg(target, 0, value, size);

		if (ERROR_OK != error_code)
			return error_code;
	}

	{
		int const error_code =
			execute_abstract_command(target, command);

		if (ERROR_OK != error_code && CMDERR_NOT_SUPPORTED == info->cmderr) {
			if (GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31) {
				info->abstract_write_fpr_supported = false;
				LOG_INFO("%s: Disabling abstract command writes to FPRs.", target_name(target));
			} else if (GDB_REGNO_CSR0 <= number && number <= GDB_REGNO_CSR4095) {
				info->abstract_write_csr_supported = false;
				LOG_INFO("%s: Disabling abstract command writes to CSRs.", target_name(target));
			}
		}

		return error_code;
	}
}

/** @return error code */
static int
examine_progbuf(struct target *const target)
{
	riscv_013_info_t *info = get_info(target);
	assert(info);

	if (YNM_MAYBE != info->progbuf_writable)
		return ERROR_OK;

	/* Figure out if progbuf is writable. */

	if (info->progbufsize < 1) {
		info->progbuf_writable = YNM_NO;
		LOG_INFO("%s: No program buffer present.", target_name(target));
		return ERROR_OK;
	}

	uint64_t s0;
	{
		int const error_code = riscv_013_register_read(target, &s0, GDB_REGNO_S0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_insert(&program, auipc(GDB_REGNO_S0));

	{
		int const error_code = riscv_program_exec(&program, target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	{
		int const error_code =
			riscv_013_register_read_direct(target, &info->progbuf_address, GDB_REGNO_S0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	riscv_program_init(&program, target);
	riscv_program_insert(&program, sw(GDB_REGNO_S0, GDB_REGNO_S0, 0));
	int const result = riscv_program_exec(&program, target);

	{
		int const error_code = riscv_013_register_write_direct(target, GDB_REGNO_S0, s0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	if (ERROR_OK != result) {
		/* This program might have failed if the program buffer is not writable. */
		info->progbuf_writable = YNM_NO;
		return ERROR_OK;
	}

	uint32_t written;

	{
		int const error_code = dmi_read(target, &written, DMI_PROGBUF0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	/**
	@todo check or eliminate cast
	*/
	info->progbuf_writable = (uint32_t)(info->progbuf_address) == written ? YNM_YES : YNM_NO;
	LOG_INFO("%s: progbuf %s at 0x%" PRIx64,
		target_name(target),
		YNM_YES == info->progbuf_writable ? "is writeable" : "is not writeable",
		info->progbuf_address);

	return ERROR_OK;
}

/** Find some scratch memory to be used with the given program. */
/** @return error code */
static int
scratch_reserve(struct target *const target,
	scratch_mem_t *const scratch,
	struct riscv_program *const program,
	unsigned const size_bytes)
{
	riscv_addr_t alignment = 1;
	while (alignment < size_bytes)
		alignment *= 2;

	assert(scratch);
	scratch->area = NULL;

	riscv_013_info_t *const info = get_info(target);
	assert(info);

	if (1 == info->dataaccess) {
		/* Sign extend dataaddr. */
		scratch->hart_address = (riscv_addr_t)(int64_t)(info->dataaddr);

		if (0 != (info->dataaddr & (1u << 11)))
			scratch->hart_address |= UINT64_C(0xFFFFFFFFFFFFF000);

		/* Align. */
		scratch->hart_address = (scratch->hart_address + alignment - 1) & ~(alignment - 1);

		if (info->datasize <= (size_bytes + scratch->hart_address - info->dataaddr + 3) / 4) {
			scratch->memory_space = SPACE_DMI_DATA;
			scratch->debug_address = (scratch->hart_address - info->dataaddr) / 4;
			return ERROR_OK;
		}
	}

	{
		int const error_code = examine_progbuf(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	/* Allow for ebreak at the end of the program. */
	assert(program);
	unsigned const program_size = (program->instruction_count + 1) * 4;
	scratch->hart_address = (info->progbuf_address + program_size + alignment - 1) & ~(alignment - 1);

	if (info->progbufsize <= (size_bytes + scratch->hart_address - info->progbuf_address + 3) / sizeof(uint32_t)) {
		scratch->memory_space = SPACE_DMI_PROGBUF;
		scratch->debug_address = (scratch->hart_address - info->progbuf_address) / 4;
		return ERROR_OK;
	}

	{
		int const error_code =
			target_alloc_working_area(target, size_bytes + alignment - 1, &scratch->area);

		if (ERROR_OK != error_code) {
			/**
			@todo Need to conform to spec minimal requirements
			*/
			LOG_ERROR("%s: Couldn't find %d bytes of scratch RAM to use."
				" Please configure a work area with 'configure -work-area-phys'.",
				target_name(target),
				size_bytes);
			return error_code;
		}
	}

	scratch->hart_address = (scratch->area->address + alignment - 1) & ~(alignment - 1);
	scratch->memory_space = SPACE_DMI_RAM;
	scratch->debug_address = scratch->hart_address;
	return ERROR_OK;
}

/** @return error code */
static int
scratch_release(struct target *const target,
	scratch_mem_t *const scratch)
{
	assert(scratch);
	return
		scratch->area ? target_free_working_area(target, scratch->area) :
		ERROR_OK;
}

/** @return error code */
static int
scratch_read64(struct target *const target,
	scratch_mem_t *const scratch,
	uint64_t *const value)
{
	uint32_t v;
	assert(scratch);

	switch (scratch->memory_space) {
	case SPACE_DMI_DATA:
		{
			int const error_code = dmi_read(target, &v, DMI_DATA0 + scratch->debug_address);

			if (ERROR_OK != error_code)
				return error_code;
		}

		assert(value);
		*value = v;

		{
			int const error_code = dmi_read(target, &v, DMI_DATA1 + scratch->debug_address);

			if (ERROR_OK != error_code)
				return error_code;
		}

		*value |= (uint64_t)(v) << 32;
		break;

	case SPACE_DMI_PROGBUF:
		{
			int const error_code =
				dmi_read(target, &v, DMI_PROGBUF0 + scratch->debug_address);

			if (ERROR_OK != error_code)
				return error_code;
		}

		assert(value);
		*value = v;

		{
			int const error_code = dmi_read(target, &v, DMI_PROGBUF1 + scratch->debug_address);

			if (ERROR_OK != error_code)
				return error_code;
		}

		*value |= (uint64_t)(v) << 32;
		break;

	case SPACE_DMI_RAM:
		{
			uint8_t buffer[8];
			{
				int const error_code = riscv_013_read_memory(target, scratch->debug_address, 4, 2, buffer);

				if (ERROR_OK != error_code)
					return error_code;
			}

			assert(value);
			*value =
				(uint64_t)(buffer[0]) << 0 * CHAR_BIT |
				(uint64_t)(buffer[1]) << 1 * CHAR_BIT |
				(uint64_t)(buffer[2]) << 2 * CHAR_BIT |
				(uint64_t)(buffer[3]) << 3 * CHAR_BIT |
				(uint64_t)(buffer[4]) << 4 * CHAR_BIT |
				(uint64_t)(buffer[5]) << 5 * CHAR_BIT |
				(uint64_t)(buffer[6]) << 6 * CHAR_BIT |
				(uint64_t)(buffer[7]) << 7 * CHAR_BIT;
		}
		break;
	}

	return ERROR_OK;
}

/** @return error code */
static int
scratch_write64(struct target *const target,
	scratch_mem_t *const scratch,
	uint64_t const value)
{
	assert(scratch);

	int error_code = ERROR_OK;

	switch (scratch->memory_space) {
	case SPACE_DMI_DATA:
		(void)(
			ERROR_OK == (error_code = dmi_write(target, DMI_DATA0 + scratch->debug_address, value)) &&
			ERROR_OK == (error_code = dmi_write(target, DMI_DATA1 + scratch->debug_address, value >> 32))
			);
		break;

	case SPACE_DMI_PROGBUF:
		(void)(
			ERROR_OK == (error_code = dmi_write(target, DMI_PROGBUF0 + scratch->debug_address, value)) &&
			ERROR_OK == (error_code = dmi_write(target, DMI_PROGBUF1 + scratch->debug_address, value >> 32))
			);
		break;

	case SPACE_DMI_RAM:
		{
			uint8_t const buffer[8] = {
				value,
				value >> 8,
				value >> 16,
				value >> 24,
				value >> 32,
				value >> 40,
				value >> 48,
				value >> 56
			};

			error_code = riscv_013_write_memory(target, scratch->debug_address, 4, 2, buffer);
		}
		break;
		/**
		@bug no default case
		*/
	}

	return error_code;
}

/** @return register size in bits. */
static unsigned
register_size(struct target *const target,
	unsigned const number)
{
	assert(target);

	/*	If reg_cache hasn't been initialized yet, make a guess.
	We need this for when this function is called during examine().
	*/
	if (target->reg_cache) {
		assert(target->reg_cache->reg_list && number < target->reg_cache->num_regs);
		return target->reg_cache->reg_list[number].size;
	} else
		return riscv_xlen(target);
}

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

	riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi && 0 <= hartid && hartid < RISCV_MAX_HARTS && num <= ('Z' - 'A'));
	return rvi->harts[hartid].misa & (1 << num);
}

/**	Checks the state of the current hart 

	-- "is_halted" checks the actual on-device register.

	@bug Uncatched errors
*/
static bool
riscv_is_halted(struct target *const target)
{
	uint32_t dmstatus;

	if (ERROR_OK != dmstatus_read(target, &dmstatus, true))
		return false;

	if (FIELD_GET(dmstatus, DMI_DMSTATUS_ANYUNAVAIL))
		LOG_ERROR("%s: Hart %d is unavailable.", target_name(target), riscv_current_hartid(target));

	if (FIELD_GET(dmstatus, DMI_DMSTATUS_ANYNONEXISTENT))
		LOG_ERROR("%s: Hart %d doesn't exist.", target_name(target), riscv_current_hartid(target));

	if (FIELD_GET(dmstatus, DMI_DMSTATUS_ANYHAVERESET)) {
		int hartid = riscv_current_hartid(target);
		LOG_INFO("%s: Hart %d unexpectedly reset!", target_name(target), hartid);
		/**
		@todo Can we make this more obvious to eg. a gdb user?
		*/
		uint32_t dmcontrol =
			DMI_DMCONTROL_DMACTIVE |
			DMI_DMCONTROL_ACKHAVERESET;

		dmcontrol = set_hartsel(dmcontrol, hartid);

		/* If we had been halted when we reset, request another halt. If we
		* ended up running out of reset, then the user will (hopefully) get a
		* message that a reset happened, that the target is running, and then
		* that it is halted again once the request goes through.
		*/
		/**
		@todo check current state but not last poll state
		*/
		if (TARGET_HALTED == target->state)
			dmcontrol |= DMI_DMCONTROL_HALTREQ;

		{
			int const error_code = dmi_write(target, DMI_DMCONTROL, dmcontrol);

			if (ERROR_OK != error_code)
				return error_code;
		}
	}

	return FIELD_GET(dmstatus, DMI_DMSTATUS_ALLHALTED);
}

/** Immediately write the new value to the requested register.

This mechanism bypasses any caches.
*/
static int
riscv_013_register_write_direct(struct target *const target,
	unsigned const number,
	uint64_t const value)
{
	LOG_DEBUG("%s: [%d] reg[0x%x] <- 0x%" PRIx64,
		target_name(target),
		riscv_current_hartid(target),
		number,
		value);

	int const result =
		register_write_abstract(target, number, value, register_size(target, number));

	assert(target);

	if (ERROR_OK == result && target->reg_cache) {
		assert(target->reg_cache->reg_list && number < target->reg_cache->num_regs);
		struct reg *const reg = &target->reg_cache->reg_list[number];
		buf_set_u64(reg->value, 0, reg->size, value);
		reg->valid = true;
	}

	riscv_013_info_t const *const info = get_info(target);
	assert(info);

	riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);

	if (ERROR_OK == result || info->progbufsize + rvi->impebreak < 2 || !riscv_is_halted(target))
		return result;

	struct riscv_program program;
	riscv_program_init(&program, target);

	uint64_t s0;
	{
		int const error_code = riscv_013_register_read(target, &s0, GDB_REGNO_S0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	scratch_mem_t scratch;
	bool use_scratch = false;

	if (GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31 &&
		riscv_supports_extension(target, riscv_current_hartid(target), 'D') &&
		riscv_xlen(target) < 64) {
		/* There are no instructions to move all the bits from a register, so
		* we need to use some scratch RAM. */
		use_scratch = true;
		riscv_program_insert(&program, fld(number - GDB_REGNO_FPR0, GDB_REGNO_S0, 0));

		{
			int const error_code = scratch_reserve(target, &scratch, &program, 8);

			if (ERROR_OK != error_code)
				return error_code;
		}

		{
			int const error_code =
				riscv_013_register_write_direct(target, GDB_REGNO_S0, scratch.hart_address);

			if (ERROR_OK != error_code) {
				scratch_release(target, &scratch);
				return error_code;
			}
		}

		{
			int const error_code = scratch_write64(target, &scratch, value);

			if (ERROR_OK != error_code) {
				scratch_release(target, &scratch);
				return error_code;
			}
		}
	} else {
		{
			int const error_code = riscv_013_register_write_direct(target, GDB_REGNO_S0, value);

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31) {
			if (riscv_supports_extension(target, riscv_current_hartid(target), 'D')) {
				int const error_code = riscv_program_insert(&program, fmv_d_x(number - GDB_REGNO_FPR0, GDB_REGNO_S0));

				if (ERROR_OK != error_code)
					return error_code;
			} else {
				int const error_code = riscv_program_insert(&program, fmv_w_x(number - GDB_REGNO_FPR0, GDB_REGNO_S0));
				if (ERROR_OK != error_code)
					return error_code;
			}
		} else if (GDB_REGNO_CSR0 <= number && number <= GDB_REGNO_CSR4095) {
			int const error_code = riscv_program_csrw(&program, GDB_REGNO_S0, number);

			if (ERROR_OK != error_code) {
				LOG_ERROR("%s: csr%d write error=%d",
					target_name(target), number - GDB_REGNO_CSR0, error_code);
				return error_code;
			}
		} else {
			LOG_ERROR("%s: Unsupported register (enum gdb_regno)(%u)",
				target_name(target), number);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	int const exec_out = riscv_program_exec(&program, target);

	/**
	@todo check Don't message on error. Probably the register doesn't exist.
	*/
	if (ERROR_OK == exec_out && target->reg_cache) {
		assert(target->reg_cache->reg_list && number < target->reg_cache->num_regs);
		struct reg *const reg = &target->reg_cache->reg_list[number];
		buf_set_u64(reg->value, 0, reg->size, value);
		reg->valid = true;
	}

	if (use_scratch)
		scratch_release(target, &scratch);

	{
		/* Restore S0. */
		int const error_code = riscv_013_register_write_direct(target, GDB_REGNO_S0, s0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	return exec_out;
}

/** @return the cached value, or read from the target if necessary. */
static int
riscv_013_register_read(struct target *const target,
	uint64_t *const value,
	uint32_t const number)
{
	if (GDB_REGNO_ZERO == number) {
		assert(value);
		*value = 0;
		return ERROR_OK;
	}

	assert(target);

	if (target->reg_cache && (number <= GDB_REGNO_XPR31 || (GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31))) {
		/* Only check the cache for registers that we know won't spontaneously change. */
		assert(target->reg_cache && target->reg_cache->reg_list && number < target->reg_cache->num_regs);
		struct reg *const reg = &target->reg_cache->reg_list[number];

		if (reg && reg->valid) {
			assert(reg->value && value);
			*value = buf_get_u64(reg->value, 0, reg->size);
			return ERROR_OK;
		}
	}

	{
		int const error_code = riscv_013_register_read_direct(target, value, number);

		if (ERROR_OK != error_code)
			return error_code;
	}

	if (target->reg_cache) {
		assert(target->reg_cache->reg_list && number < target->reg_cache->num_regs);
		struct reg *const reg = &target->reg_cache->reg_list[number];
		assert(reg->value && value);
		buf_set_u64(reg->value, 0, reg->size, *value);
		reg->valid = true;
	}

	return ERROR_OK;
}

/** Actually read registers from the target right now. */
/** @return error code */
static int
riscv_013_register_read_direct(struct target *const target,
	uint64_t *const value,
	uint32_t const number)
{
	int result =
		register_read_abstract(target, value, number, register_size(target, number));

	riscv_013_info_t *const info = get_info(target);
	assert(info);

	riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);

	if (ERROR_OK != result && 2 <= info->progbufsize + rvi->impebreak && GDB_REGNO_XPR31 < number) {
		struct riscv_program program;
		riscv_program_init(&program, target);

		scratch_mem_t scratch;
		bool use_scratch = false;

		uint64_t s0;
		{
			int const error_code = riscv_013_register_read(target, &s0, GDB_REGNO_S0);

			if (ERROR_OK != error_code)
				return result;
		}

		/* Write program to move data into s0. */
		uint64_t mstatus;

		if (GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31) {
			if (
				ERROR_OK != riscv_013_register_read(target, &mstatus, GDB_REGNO_MSTATUS) ||
				(
					0 == (mstatus & MSTATUS_FS) &&
					ERROR_OK != riscv_013_register_write_direct(target, GDB_REGNO_MSTATUS, FIELD_SET(mstatus, MSTATUS_FS, 1))
					)
				)
				return result;

			if (riscv_supports_extension(target, riscv_current_hartid(target), 'D') &&
				riscv_xlen(target) < 64) {
				/* There are no instructions to move all the bits from a
				* register, so we need to use some scratch RAM. */
				(void)(riscv_program_insert(&program, fsd(number - GDB_REGNO_FPR0, GDB_REGNO_S0, 0)));

				if (ERROR_OK != scratch_reserve(target, &scratch, &program, 8))
					return result;

				use_scratch = true;

				if (ERROR_OK != riscv_013_register_write_direct(target, GDB_REGNO_S0, scratch.hart_address)) {
					(void)(scratch_release(target, &scratch));
					return result;
				}
			} else if (riscv_supports_extension(target, riscv_current_hartid(target), 'D')) {
				(void)(riscv_program_insert(&program, fmv_x_d(GDB_REGNO_S0, number - GDB_REGNO_FPR0)));
			} else {
				(void)(riscv_program_insert(&program, fmv_x_w(GDB_REGNO_S0, number - GDB_REGNO_FPR0)));
			}
		} else if (GDB_REGNO_CSR0 <= number && number <= GDB_REGNO_CSR4095) {
			if (ERROR_OK != (result = riscv_program_csrr(&program, GDB_REGNO_S0, number)))
				return result;
		} else {
			LOG_ERROR("%s: Unsupported register (enum gdb_regno)(%u)",
				target_name(target), number);
			return result;
		}

		/*
		Execute program.
		Don't message on error.
		Probably the register doesn't exist.
		*/
		result = riscv_program_exec(&program, target);

		if (use_scratch) {
			result = scratch_read64(target, &scratch, value);
			(void)(scratch_release(target, &scratch));

			if (ERROR_OK != result)
				return result;
		} else {
			/* Read S0 */
			int const error_code = riscv_013_register_read_direct(target, value, GDB_REGNO_S0);

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31 && 0 == (mstatus & MSTATUS_FS)) {
			int const error_code = riscv_013_register_write_direct(target, GDB_REGNO_MSTATUS, mstatus);

			if (ERROR_OK != error_code)
				return error_code;
		}

		{
			/* Restore S0. */
			int const error_code = riscv_013_register_write_direct(target, GDB_REGNO_S0, s0);

			if (ERROR_OK != error_code)
				return error_code;
		}
	}

	if (ERROR_OK == result) {
		LOG_DEBUG("%s: [%d] reg[0x%x] = 0x%" PRIx64, target_name(target), riscv_current_hartid(target),
			number, *value);
	}

	return result;
}

/** @return error code */
static int
wait_for_authbusy(struct target *const target, uint32_t *dmstatus)
{
	time_t const start = time(NULL);

	for (;;) {
		uint32_t value;
		{
			int const error_code = dmstatus_read(target, &value, false);

			if (ERROR_OK == error_code)
				return error_code;
		}

		if (dmstatus)
			*dmstatus = value;

		if (!FIELD_GET(value, DMI_DMSTATUS_AUTHBUSY))
			break;

		if (start + riscv_command_timeout_sec < time(NULL)) {
			LOG_ERROR("%s: Timed out after %ds waiting for authbusy to go low (dmstatus=0x%x). "
				"Increase the timeout with riscv set_command_timeout_sec.",
				target_name(target), riscv_command_timeout_sec, value);
			return ERROR_TARGET_TIMEOUT;
		}
	}

	return ERROR_OK;
}

static void
riscv_013_deinit_target(struct target *const target)
{
	LOG_DEBUG("%s: riscv_deinit_target()", target_name(target));
	riscv_info_t *const info = target->arch_info;
	assert(info);
	free(info->version_specific);
	/**
	@todo free register arch_info
	*/
	info->version_specific = NULL;
}

/** @return error code */
static int
riscv_013_select_current_hart(struct target *const target)
{
	dm013_info_t *const dm = get_dm(target);
	riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi && dm);

	if (rvi->current_hartid == dm->current_hartid)
		return ERROR_OK;

	uint32_t dmcontrol;

	{
		/**
		@todo can't we just "dmcontrol = DMI_DMACTIVE"?
		*/
		int const error_code = dmi_read(target, &dmcontrol, DMI_DMCONTROL);

		if (ERROR_OK != error_code)
			return error_code;
	}

	dmcontrol = set_hartsel(dmcontrol, rvi->current_hartid);
	int const result = dmi_write(target, DMI_DMCONTROL, dmcontrol);

	dm->current_hartid = rvi->current_hartid;
	return result;
}

/** @return error code */
static int
riscv_013_halt_current_hart(struct target *const target)
{
	riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);
	LOG_DEBUG("%s: halting hart %d", target_name(target), rvi->current_hartid);

	if (riscv_is_halted(target))
		LOG_ERROR("%s: Hart %d is already halted!", target_name(target), rvi->current_hartid);

	/* Issue the halt command, and then wait for the current hart to halt. */
	uint32_t dmcontrol;

	{
		int const error_code = dmi_read(target, &dmcontrol, DMI_DMCONTROL);

		if (ERROR_OK != error_code)
			return error_code;
	}

	dmcontrol = FIELD_SET(dmcontrol, DMI_DMCONTROL_HALTREQ, 1);
	{
		int const error_code = dmi_write(target, DMI_DMCONTROL, dmcontrol);

		if (ERROR_OK != error_code)
			return error_code;
	}

	for (size_t i = 0; i < 256; ++i) {
		if (riscv_is_halted(target))
			break;
	}

	if (!riscv_is_halted(target)) {
		uint32_t dmstatus;

		{
			int const error_code = dmstatus_read(target, &dmstatus, true);

			if (ERROR_OK != error_code)
				return error_code;
		}

		{
			int const error_code = dmi_read(target, &dmcontrol, DMI_DMCONTROL);

			if (ERROR_OK != error_code)
				return error_code;
		}

		/**
		@todo Make single message
		*/
		LOG_ERROR("%s: unable to halt hart %d dmcontrol=0x%08" PRIx32 " dmstatus =0x%08" PRIx32,
			target_name(target), rvi->current_hartid, dmcontrol, dmstatus);
		return ERROR_TARGET_FAILURE;
	}

	dmcontrol = FIELD_SET(dmcontrol, DMI_DMCONTROL_HALTREQ, 0);
	return dmi_write(target, DMI_DMCONTROL, dmcontrol);
}

/** @return error code */
static int
riscv_013_authdata_read(struct target *const target,
	uint32_t *const value)
{
	int const error_code = wait_for_authbusy(target, NULL);
	return
		ERROR_OK != error_code ? error_code :
		dmi_read(target, value, DMI_AUTHDATA);
}

static inline bool
__attribute__((pure))
riscv_rtos_enabled(struct target const *const target)
{
	assert(target);
	return !!target->rtos;
}

/** @returns TRUE when a hart is enabled in this target. */
static bool
riscv_hart_enabled(struct target *const target,
	int hartid)
{
	/**
	@todo FIXME: Add a hart mask to the RTOS.
	*/
	if (riscv_rtos_enabled(target))
		return hartid < riscv_count_harts(target);

	return hartid == target->coreid;
}

/** @return error code */
static int
riscv_013_assert_reset(struct target *const target)
{
	select_dmi(target->tap);

	uint32_t const control_base = FIELD_SET(0, DMI_DMCONTROL_DMACTIVE, 1);
	assert(target);

	if (target->rtos) {
		/* There's only one target, and OpenOCD thinks each hart is a thread.
		We must reset them all. */

		/**
		@todo Try to use hasel in dmcontrol
		*/

		/* Set haltreq for each hart. */
		uint32_t control = control_base;

		for (int i = 0; i < riscv_count_harts(target); ++i) {
			if (!riscv_hart_enabled(target, i))
				continue;

			control =
				FIELD_SET(set_hartsel(control_base, i),
					DMI_DMCONTROL_HALTREQ, target->reset_halt ? 1 : 0);
			int const error_code = dmi_write(target, DMI_DMCONTROL, control);

			if (ERROR_OK != error_code)
				return error_code;
		}

		/* Assert ndmreset */
		control = FIELD_SET(control, DMI_DMCONTROL_NDMRESET, 1);
		int const error_code = dmi_write(target, DMI_DMCONTROL, control);

		if (ERROR_OK != error_code)
			return error_code;
	} else {
		/* Reset just this hart. */
		riscv_info_t const *const rvi = riscv_info(target);
		assert(rvi);
		uint32_t control =
			FIELD_SET(
				FIELD_SET(
					set_hartsel(control_base, rvi->current_hartid),
					DMI_DMCONTROL_HALTREQ, target->reset_halt ? 1 : 0),
				DMI_DMCONTROL_NDMRESET, 1);

		int const error_code = dmi_write(target, DMI_DMCONTROL, control);

		if (ERROR_OK != error_code)
			return error_code;
	}

	/**
	@todo target callback
	*/
	target->state = TARGET_RESET;

	return ERROR_OK;
}

/** @return error code */
static int
riscv_013_deassert_reset(struct target *const target)
{
	assert(target);
	LOG_DEBUG("%s: RISCV DEASSERT RESET", target_name(target));
	riscv_013_info_t *const info = get_info(target);
	select_dmi(target->tap);

	/* Clear the reset, but make sure haltreq is still set */
	uint32_t control =
		FIELD_SET(
			FIELD_SET(0, DMI_DMCONTROL_HALTREQ, target->reset_halt ? 1 : 0),
			DMI_DMCONTROL_DMACTIVE, 1);
	riscv_info_t const *const rvi = riscv_info(target);
	{
		int const error_code = dmi_write(target, DMI_DMCONTROL, set_hartsel(control, rvi->current_hartid));

		if (ERROR_OK != error_code)
			return error_code;
	}

	uint32_t dmstatus;
	assert(info);
	int dmi_busy_delay = info->dmi_busy_delay;
	time_t const start = time(NULL);

	for (int i = 0; i < riscv_count_harts(target); ++i) {
		int index = i;

		if (target->rtos) {
			if (!riscv_hart_enabled(target, index))
				continue;

			int const error_code =
				dmi_write(target, DMI_DMCONTROL, set_hartsel(control, index));

			if (ERROR_OK != error_code)
				return error_code;
		} else {
			index = rvi->current_hartid;
		}

		char const *const operation = target->reset_halt ? "halt" : "run";
		uint32_t const expected_field =
			target->reset_halt ? DMI_DMSTATUS_ALLHALTED : DMI_DMSTATUS_ALLRUNNING;

		LOG_DEBUG("%s: Waiting for hart %d to %s out of reset.", target_name(target), index, operation);

		for (;;) {
			int const result =
				dmstatus_read_timeout(target, &dmstatus, true, riscv_reset_timeout_sec);

			if (ERROR_TARGET_TIMEOUT == result)
				LOG_ERROR("%s: Hart %d didn't complete a DMI read coming out of reset in %ds;"
					" Increase the timeout with riscv set_reset_timeout_sec.",
					target_name(target), index, riscv_reset_timeout_sec);

			if (ERROR_OK != result)
				return result;

			if (FIELD_GET(dmstatus, expected_field))
				break;

			if (start + riscv_reset_timeout_sec < time(NULL)) {
				LOG_ERROR("%s: Hart %d didn't %s coming out of reset in %ds;"
					" dmstatus=0x%x; Increase the timeout with riscv set_reset_timeout_sec.",
					target_name(target), index, operation, riscv_reset_timeout_sec, dmstatus);
				return ERROR_TARGET_TIMEOUT;
			}
		}

		/**
		@todo target callback
		*/
		target->state = TARGET_HALTED;

		if (FIELD_GET(dmstatus, DMI_DMSTATUS_ALLHAVERESET)) {
			/* Ack reset. */
			int const error_code =
				dmi_write(target,
					DMI_DMCONTROL,
					DMI_DMCONTROL_ACKHAVERESET | set_hartsel(control, index));

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (!target->rtos)
			break;
	}

	info->dmi_busy_delay = dmi_busy_delay;
	return ERROR_OK;
}

static void
write_to_buf(uint8_t *const buffer,
	uint64_t const value,
	unsigned const size /**<[in] size in bytes */)
{
	switch (size) {
	case 8:
		buffer[7] = (uint8_t)(value >> 7 * CHAR_BIT);
		buffer[6] = (uint8_t)(value >> 6 * CHAR_BIT);
		buffer[5] = (uint8_t)(value >> 5 * CHAR_BIT);
		buffer[4] = (uint8_t)(value >> 4 * CHAR_BIT);
		/* falls through */

	case 4:
		buffer[3] = (uint8_t)(value >> 3 * CHAR_BIT);
		buffer[2] = (uint8_t)(value >> 2 * CHAR_BIT);
		/* falls through */

	case 2:
		buffer[1] = (uint8_t)(value >> 1 * CHAR_BIT);
		/* falls through */

	case 1:
		buffer[0] = (uint8_t)(value >> 0 * CHAR_BIT);
		break;

	default:
		assert(false);
	}
}

/** Invalidates the register cache. */
static void
riscv_invalidate_register_cache(struct target *const target)
{
	register_cache_invalidate(target->reg_cache);

	for (size_t i = 0; i < target->reg_cache->num_regs; ++i) {
		struct reg *reg = &target->reg_cache->reg_list[i];
		assert(reg);
		reg->valid = false;
	}

	riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);
	rvi->registers_initialized = true;
}

/**	Sets the current hart,
which is the hart that will actually be used when issuing debug commands.

@return error code
*/
static int
riscv_set_current_hartid(struct target *const target,
	int const hartid)
{
	riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);

	int const previous_hartid = riscv_current_hartid(target);
	rvi->current_hartid = hartid;
	assert(riscv_hart_enabled(target, hartid));
	LOG_DEBUG("%s: setting hartid to %d, was %d", target_name(target), hartid, previous_hartid);

	{
		int const error_code = riscv_013_select_current_hart(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	/* This might get called during init, in which case we shouldn't be
	* setting up the register cache. */
	if (!target_was_examined(target))
		/**
		@todo ERROR_TARGET_NOT_EXAMINED
		*/
		return ERROR_OK;

	riscv_invalidate_register_cache(target);
	return ERROR_OK;
}

/** @return error code */
static int
execute_fence(struct target *const target)
{
	int old_hartid = riscv_current_hartid(target);

	/**
	@todo FIXME: For non-coherent systems we need to flush the caches right here,
	but there's no ISA-defined way of doing that.
	*/
	{
		struct riscv_program program;
		riscv_program_init(&program, target);
		riscv_program_fence_i(&program);
		riscv_program_fence(&program);
		int const error_code = riscv_program_exec(&program, target);

		if (ERROR_OK != error_code)
			LOG_DEBUG("%s: Unable to execute pre-fence", target_name(target));
	}

	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		riscv_set_current_hartid(target, i);

		struct riscv_program program;
		riscv_program_init(&program, target);
		riscv_program_fence_i(&program);
		riscv_program_fence(&program);
		{
			int const error_code = riscv_program_exec(&program, target);

			if (ERROR_OK != error_code)
				LOG_DEBUG("%s: Unable to execute fence on hart %d",
					target_name(target), i);
		}
	}

	return riscv_set_current_hartid(target, old_hartid);
}

static void
log_memory_access(target_addr_t const address,
	uint64_t const value,
	unsigned const size_bytes,
	bool const read)
{
	if (debug_level < LOG_LVL_DEBUG)
		return;

	/**
	@todo Improve mask
	*/
	uint64_t const mask = (UINT64_C(0x1) << (size_bytes * CHAR_BIT)) - 1;
	LOG_DEBUG("M[0x%" TARGET_PRIxADDR "] %ss 0x%0*" PRIx64,
		address,
		read ? "read" : "write",
		size_bytes * 2,
		mask & value);
}

/**
Read the relevant @c sbdata regs depending on size, and put the results into buffer. */
/** @return error code */
static int
read_memory_bus_word(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint8_t *const buffer)
{
	uint32_t value;

	if (12 < size) {
		{
			int const error_code = dmi_read(target, &value, DMI_SBDATA3);

			if (ERROR_OK != error_code)
				return error_code;
		}

		write_to_buf(buffer + 12, value, 4);
		log_memory_access(address + 12, value, 4, true);
	}

	if (8 < size) {
		{
			int const error_code = dmi_read(target, &value, DMI_SBDATA2);

			if (ERROR_OK != error_code)
				return error_code;
		}

		write_to_buf(buffer + 8, value, 4);
		log_memory_access(address + 8, value, 4, true);
	}

	if (4 < size) {
		{
			int const error_code = dmi_read(target, &value, DMI_SBDATA1);

			if (ERROR_OK != error_code)
				return error_code;
		}

		write_to_buf(buffer + 4, value, 4);
		log_memory_access(address + 4, value, 4, true);
	}

	{
		int const error_code = dmi_read(target, &value, DMI_SBDATA0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	write_to_buf(buffer, value, MIN(size, 4));
	log_memory_access(address, value, MIN(size, 4), true);
	return ERROR_OK;
}

static uint32_t
sb_sbaccess(unsigned size_bytes)
{
	switch (size_bytes) {
	case 1:
		return FIELD_SET(0, DMI_SBCS_SBACCESS, 0);
	case 2:
		return FIELD_SET(0, DMI_SBCS_SBACCESS, 1);
	case 4:
		return FIELD_SET(0, DMI_SBCS_SBACCESS, 2);
	case 8:
		return FIELD_SET(0, DMI_SBCS_SBACCESS, 3);
	case 16:
		return FIELD_SET(0, DMI_SBCS_SBACCESS, 4);
	default:
		assert(0);
		return 0;
	}
}

static target_addr_t
sb_read_address(struct target *const target)
{
	riscv_013_info_t *const info = get_info(target);
	assert(info);
	unsigned sbasize = FIELD_GET(info->sbcs, DMI_SBCS_SBASIZE);
	target_addr_t address = 0;
#if BUILD_TARGET64
	if (sbasize > 32) {
		uint32_t v;
		{
			int const error_code = dmi_read(target, &v, DMI_SBADDRESS1);

			if (ERROR_OK != error_code) {
				LOG_ERROR("%s: dmi_read return status %d, sb_read_address return invalid data",
					target_name(target), error_code);
				return 0xBADC0DE;
			}
		}

		address |= v;
		address <<= 32;
	}
#endif
	{
		uint32_t v;
		{
			int const error_code = dmi_read(target, &v, DMI_SBADDRESS0);

			if (ERROR_OK != error_code) {
				LOG_ERROR("%s: dmi_read return status %d, sb_read_address return invalid data",
					target_name(target), error_code);
				return 0xBADC0DE;
			}
		}

		address |= v;
	}

	return address;
}

/** @return error code */
static int
sb_write_address(struct target *const target,
	target_addr_t const address)
{
	riscv_013_info_t *const info = get_info(target);
	assert(info);
	unsigned sbasize = FIELD_GET(info->sbcs, DMI_SBCS_SBASIZE);

	/* There currently is no support for >64-bit addresses in OpenOCD. */
	if (sbasize > 96) {
		int const error_code = dmi_write(target, DMI_SBADDRESS3, 0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	if (sbasize > 64) {
		int const error_code = dmi_write(target, DMI_SBADDRESS2, 0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	if (sbasize > 32) {
#if BUILD_TARGET64
		int const error_code = dmi_write(target, DMI_SBADDRESS1, address >> 32);
#else
		int const error_code = dmi_write(target, DMI_SBADDRESS1, 0);
#endif

		if (ERROR_OK != error_code)
			return error_code;
	}

	return dmi_write(target, DMI_SBADDRESS0, address);
}

/** @return error code */
static int
read_sbcs_nonbusy(struct target *const target,
	uint32_t *const sbcs /**<[out]*/)
{
	assert(sbcs);
	time_t const start = time(NULL);

	for (;;) {
		{
			int const error_code = dmi_read(target, sbcs, DMI_SBCS);

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (0 == FIELD_GET(*sbcs, DMI_SBCS_SBBUSY))
			return ERROR_OK;

		if (start + riscv_command_timeout_sec < time(NULL)) {
			LOG_ERROR("%s: Timed out after %ds waiting for sbbusy to go low (sbcs=0x%x)."
				" Increase the timeout with riscv set_command_timeout_sec.",
				target_name(target), riscv_command_timeout_sec, *sbcs);
			return ERROR_TARGET_TIMEOUT;
		}
	}
}

/** @return error code */
static int
read_memory_bus_v0(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint32_t const count,
	uint8_t *const buffer)
{
	LOG_DEBUG("%s: System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
		TARGET_PRIxADDR, target_name(target), size, count, address);
	uint8_t *t_buffer = buffer;
	riscv_addr_t cur_addr = address;
	riscv_addr_t const fin_addr = address + (count * size);
	uint32_t access = 0;

	enum {
		DMI_SBCS_SBSINGLEREAD_OFFSET = 20,
		DMI_SBCS_SBAUTOREAD_OFFSET = 15,

	};
	static uint32_t const DMI_SBCS_SBSINGLEREAD = UINT32_C(1) << DMI_SBCS_SBSINGLEREAD_OFFSET;
	static uint32_t const DMI_SBCS_SBAUTOREAD = UINT32_C(1) << DMI_SBCS_SBAUTOREAD_OFFSET;

	/* ww favorise one off reading if there is an issue */
	if (1 == count) {
		for (uint32_t i = 0; i < count; ++i) {
			{
				int const error_code = dmi_read(target, &access, DMI_SBCS);

				if (ERROR_OK != error_code)
					return error_code;
			}
			{
				int const error_code = dmi_write(target, DMI_SBADDRESS0, cur_addr);

				if (ERROR_OK != error_code)
					return error_code;
			}
			/* size/2 matching the bit access of the spec 0.13 */
			access =
				FIELD_SET(
					FIELD_SET(access, DMI_SBCS_SBACCESS, size / 2),
					DMI_SBCS_SBSINGLEREAD,
					1);
			LOG_DEBUG("%s: read_memory: sab: access:  0x%08" PRIx32,
				target_name(target), access);
			{
				int const error_code = dmi_write(target, DMI_SBCS, access);

				if (ERROR_OK != error_code)
					return error_code;
			}
			/* 3) read */
			uint32_t value;
			{
				int const error_code = dmi_read(target, &value, DMI_SBDATA0);

				if (ERROR_OK != error_code)
					return error_code;
			}
			LOG_DEBUG("%s: read_memory: sab: value:  0x%08x", target_name(target), value);
			write_to_buf(t_buffer, value, size);
			t_buffer += size;
			cur_addr += size;
		}

		return ERROR_OK;
	} else {
		/* has to be the same size if we want to read a block */
		LOG_DEBUG("%s: reading block until final address 0x%" PRIx64, target_name(target), fin_addr);

		{
			int const error_code = dmi_read(target, &access, DMI_SBCS);

			if (ERROR_OK != error_code)
				return error_code;
		}

		{
			/* set current address */
			int const error_code = dmi_write(target, DMI_SBADDRESS0, cur_addr);

			if (ERROR_OK != error_code)
				return error_code;
		}
		/* 2) write sbaccess=2, sbsingleread, sbautoread, sbautoincrement
		* size/2 matching the bit access of the spec 0.13 */
		access =
			FIELD_SET(
				FIELD_SET(
					FIELD_SET(
						FIELD_SET(access,
							DMI_SBCS_SBACCESS, size / 2),
						DMI_SBCS_SBAUTOREAD, 1),
					DMI_SBCS_SBSINGLEREAD, 1),
				DMI_SBCS_SBAUTOINCREMENT, 1);
		LOG_DEBUG("%s: access:  0x%08x",
			target_name(target), access);
		{
			int const error_code = dmi_write(target, DMI_SBCS, access);

			if (ERROR_OK != error_code)
				return error_code;
		}

		while (cur_addr < fin_addr) {
			LOG_DEBUG("%s"
				": sab:autoincrement:\tsize: %" PRId32
				"\tcount:%" PRId32
				"\taddress: 0x%08" PRIx64,
				target_name(target),
				size,
				count,
				cur_addr);

			/* read */
			uint32_t value;

			{
				int const error_code = dmi_read(target, &value, DMI_SBDATA0);

				if (ERROR_OK != error_code)
					return error_code;
			}

			write_to_buf(t_buffer, value, size);
			cur_addr += size;
			t_buffer += size;

			/* if we are reaching last address, we must clear autoread */
			if (cur_addr == fin_addr && count != 1) {
				int error_code;

				if (!(
					ERROR_OK == (error_code = dmi_write(target, DMI_SBCS, 0)) &&
					ERROR_OK == (error_code = dmi_read(target, &value, DMI_SBDATA0))
					))
					return error_code;

				write_to_buf(t_buffer, value, size);
			}
		}

		return ERROR_OK;
	}
}

/**
* Read the requested memory using the system bus interface.
*/
static int
read_memory_bus_v1(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint32_t const count,
	uint8_t *const buffer)
{
	riscv_013_info_t *const info = get_info(target);
	assert(info);

	target_addr_t const end_address = address + count * size;

	for (target_addr_t next_address = address; next_address < end_address;) {
		{
			uint32_t const sbcs =
				FIELD_SET(
					FIELD_SET(
						FIELD_SET(0, DMI_SBCS_SBREADONADDR, 1) | sb_sbaccess(size),
						DMI_SBCS_SBAUTOINCREMENT,
						1),
					DMI_SBCS_SBREADONDATA,
					1 < count);
			{
				int const error_code = dmi_write(target, DMI_SBCS, sbcs);

				if (ERROR_OK != error_code)
					return error_code;
			}

			/* This address write will trigger the first read. */
			/**
			@todo check return
			*/
			sb_write_address(target, next_address);

			if (0 < info->bus_master_read_delay) {
				jtag_add_runtest(info->bus_master_read_delay, TAP_IDLE);

				{
					int const error_code = jtag_execute_queue();

					if (ERROR_OK != error_code) {
						LOG_ERROR("%s: Failed to scan idle sequence", target_name(target));
						return error_code;
					}
				}
			}

			for (uint32_t i = (next_address - address) / size; i < count - 1; ++i)
				read_memory_bus_word(target, address + i * size, size, buffer + i * size);

			{
				int const error_code = dmi_write(target, DMI_SBCS, FIELD_SET(sbcs, DMI_SBCS_SBREADONDATA, 0));

				if (ERROR_OK != error_code)
					return error_code;
			}
		}

		read_memory_bus_word(target, address + (count - 1) * size, size, buffer + (count - 1) * size);

		{
			uint32_t sbcs_1;
			{
				int const error_code = read_sbcs_nonbusy(target, &sbcs_1);

				if (ERROR_OK != error_code)
					return error_code;
			}

			if (0 != FIELD_GET(sbcs_1, DMI_SBCS_SBBUSYERROR)) {
				/* We read while the target was busy. Slow down and try again. */
				{
					int const error_code = dmi_write(target, DMI_SBCS, DMI_SBCS_SBBUSYERROR);

					if (ERROR_OK != error_code)
						return error_code;
				}
				next_address = sb_read_address(target);
				info->bus_master_read_delay += info->bus_master_read_delay / 10 + 1;
				continue;
			}

			if (0 != FIELD_GET(sbcs_1, DMI_SBCS_SBERROR)) {
				/* Some error indicating the bus access failed,
				but not because of something we did wrong. */
				LOG_WARNING("%s: sbcs error %" PRIx32,
					target_name(target), FIELD_GET(sbcs_1, DMI_SBCS_SBERROR));
				int const error = dmi_write(target, DMI_SBCS, DMI_SBCS_SBERROR);
				(void)(error);
				return ERROR_TARGET_FAILURE;
			}
		}

		next_address = end_address;
	}

	return ERROR_OK;
}

static int
riscv_013_clear_abstract_error(struct target *const target)
{
	/* Wait for busy to go away. */
	time_t const start = time(NULL);
	uint32_t abstractcs;

	for (;;) {
		{
			int const error_code = dmi_read(target, &abstractcs, DMI_ABSTRACTCS);

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (0 == FIELD_GET(abstractcs, DMI_ABSTRACTCS_BUSY))
			break;

		if (time(NULL) > start + riscv_command_timeout_sec) {
			LOG_ERROR("%s: abstractcs.busy is not going low after %d seconds "
				"(abstractcs=0x%x). The target is either really slow or "
				"broken. You could increase the timeout with riscv "
				"set_command_timeout_sec.", target_name(target),
				riscv_command_timeout_sec, abstractcs);
			break;
		}
	}

	/* Clear the error status. */
	return dmi_write(target, DMI_ABSTRACTCS, abstractcs & DMI_ABSTRACTCS_CMDERR);
}

/**
Read the requested memory, taking care to execute every read exactly once,
even if cmderr=busy is encountered.
*/
/** @return error code */
static int
read_memory_progbuf(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint32_t const count,
	uint8_t *const buffer)
{
	riscv_013_info_t *const info = get_info(target);

	LOG_DEBUG("%s: reading %d words of %d bytes from 0x%" TARGET_PRIxADDR,
		target_name(target), count, size, address);

	select_dmi(target->tap);

	/*	s0 holds the next address to write to
	s1 holds the next data value to write
	*/
	uint64_t s0;
	{
		int const error_code = riscv_013_register_read(target, &s0, GDB_REGNO_S0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	uint64_t s1;
	{
		int const error_code = riscv_013_register_read(target, &s1, GDB_REGNO_S1);

		if (ERROR_OK != error_code)
			return error_code;
	}

	{
		int const error_code = execute_fence(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	/* Write the program (load, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);

	switch (size) {
	case 1:
		riscv_program_lbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
		break;

	case 2:
		riscv_program_lhr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
		break;

	case 4:
		riscv_program_lwr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
		break;

	default:
		LOG_ERROR("%s: Unsupported size: %d", target_name(target), size);
		return ERROR_TARGET_INVALID;
	}

	{
		int const error_code = riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);
		if (ERROR_OK != error_code)
			return error_code;
	}

	{
		int const error_code = riscv_program_ebreak(&program);
		if (ERROR_OK != error_code)
			return error_code;
	}

	{
		int const error_code = riscv_program_write(&program);
		if (ERROR_OK != error_code)
			return error_code;
	}

	/* Write address to S0, and execute buffer. */
	int result = riscv_013_register_write_direct(target, GDB_REGNO_S0, address);
	if (ERROR_OK != result)
		goto error;

	uint32_t const command =
		access_register_command(target,
			GDB_REGNO_S1,
			riscv_xlen(target),
			AC_ACCESS_REGISTER_TRANSFER | AC_ACCESS_REGISTER_POSTEXEC);

	if (ERROR_OK != (result = execute_abstract_command(target, command)))
		goto error;

	/* First read has just triggered. Result is in s1. */

	result =
		dmi_write(target, DMI_ABSTRACTAUTO, 1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);

	if (ERROR_OK != result)
		goto error;

	/* read_addr is the next address that the hart will read from, which is the
	* value in s0. */
	riscv_addr_t read_addr = address + size;
	/* The next address that we need to receive data for. */
	riscv_addr_t receive_addr = address;
	riscv_addr_t const fin_addr = address + (count * size);
	unsigned skip = 1;

	while (read_addr < fin_addr) {
		LOG_DEBUG("%s: read_addr=0x%" PRIx64 ", receive_addr=0x%" PRIx64 ", fin_addr=0x%" PRIx64,
			target_name(target), read_addr, receive_addr, fin_addr);

		/* The pipeline looks like this:
		memory -> s1 -> dm_data0 -> debugger

		It advances every time the debugger reads dmdata0.

		So at any time the debugger has just read mem[s0 - 3*size],
		dm_data0 contains mem[s0 - 2*size]
		s1 contains mem[s0-size]
		*/

		LOG_DEBUG("%s: creating burst to read from 0x%" PRIx64 " up to 0x%" PRIx64,
			target_name(target), read_addr, fin_addr);
		assert(address <= read_addr && read_addr < fin_addr);
		struct riscv_batch *const batch =
			riscv_batch_alloc(target, 32, info->dmi_busy_delay + info->ac_busy_delay);
		size_t reads = 0;

		for (riscv_addr_t addr = read_addr; addr < fin_addr; addr += size) {
			riscv_batch_add_dmi_read(batch, DMI_DATA0);
			++reads;

			if (riscv_batch_full(batch))
				break;
		}

		if (ERROR_OK != (result = riscv_batch_run(batch)))
			goto error;

		/*	Wait for the target to finish performing the last abstract command,
		and update our copy of cmderr. */
		uint32_t abstractcs;

		do {
			if (ERROR_OK != (result = dmi_read(target, &abstractcs, DMI_ABSTRACTCS)))
				goto error;
		} while (0 != FIELD_GET(abstractcs, DMI_ABSTRACTCS_BUSY));

		info->cmderr = FIELD_GET(abstractcs, DMI_ABSTRACTCS_CMDERR);

		unsigned const cmderr = info->cmderr;
		riscv_addr_t next_read_addr;
		uint32_t dmi_data0 = -1;

		switch (info->cmderr) {
		case CMDERR_NONE:
			{
				LOG_DEBUG("%s: successful (partial?) memory read", target_name(target));
				next_read_addr = read_addr + reads * size;
			}
			break;

		case CMDERR_BUSY:
			{
				LOG_DEBUG("%s: memory read resulted in busy response", target_name(target));

				increase_ac_busy_delay(target);
				riscv_013_clear_abstract_error(target);

				if (ERROR_OK != (result = dmi_write(target, DMI_ABSTRACTAUTO, 0)))
					goto error;

				/* This is definitely a good version of the value that we
				* attempted to read when we discovered that the target was
				* busy. */
				if (ERROR_OK != (result = dmi_read(target, &dmi_data0, DMI_DATA0))) {
					riscv_batch_free(batch);
					goto error;
				}

				/* Clobbers DMI_DATA0. */
				if (ERROR_OK != (result = riscv_013_register_read_direct(target, &next_read_addr, GDB_REGNO_S0))) {
					riscv_batch_free(batch);
					goto error;
				}

				/*
				Restore the command, and execute it.
				Now DMI_DATA0 contains the next value just as it would if no error had occurred.
				*/
				if (!(
					ERROR_OK == (result = dmi_write(target, DMI_COMMAND, command)) &&
					ERROR_OK == (result = dmi_write(target, DMI_ABSTRACTAUTO, 1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET))
					))
					goto error;
			}
			break;

		default:
			{
				LOG_ERROR("%s: error when reading memory, abstractcs=0x%08" PRIx32,
					target_name(target), abstractcs);
				riscv_013_clear_abstract_error(target);
				riscv_batch_free(batch);
				result = ERROR_TARGET_FAILURE;
			}
			goto error;
		}

		/* Now read whatever we got out of the batch. */
		for (size_t i = 0; i < reads; ++i) {
			if (next_read_addr <= read_addr)
				break;

			read_addr += size;

			if (0 < skip) {
				--skip;
				continue;
			}

			riscv_addr_t const offset = receive_addr - address;
			uint64_t const dmi_out = riscv_batch_get_dmi_read(batch, i);
			uint32_t const value = FIELD_GET(dmi_out, DTM_DMI_DATA);
			write_to_buf(buffer + offset, value, size);
			log_memory_access(receive_addr, value, size, true);

			receive_addr += size;
		}

		riscv_batch_free(batch);

		if (CMDERR_BUSY == cmderr) {
			riscv_addr_t const offset = receive_addr - address;
			write_to_buf(buffer + offset, dmi_data0, size);
			log_memory_access(receive_addr, dmi_data0, size, true);
			read_addr += size;
			receive_addr += size;
		}
	}

	if (ERROR_OK != (result = dmi_write(target, DMI_ABSTRACTAUTO, 0)))
		goto error;

	if (count > 1) {
		/* Read the penultimate word. */
		uint32_t value;

		if (ERROR_OK != (result = dmi_read(target, &value, DMI_DATA0)))
			goto error;

		write_to_buf(buffer + receive_addr - address, value, size);
		log_memory_access(receive_addr, value, size, true);
		receive_addr += size;
	}

	{
		/* Read the last word. */
		uint64_t value;
		if (ERROR_OK != (result = riscv_013_register_read_direct(target, &value, GDB_REGNO_S1)))
			goto error;

		write_to_buf(buffer + receive_addr - address, value, size);
		log_memory_access(receive_addr, value, size, true);
	}

	goto finish;

error:
	{
		LOG_ERROR("%s: read_memory_progbuf error (%d)", target_name(target), result);
		int const error_code = dmi_write(target, DMI_ABSTRACTAUTO, 0);
		(void)(error_code);
	}

finish:
	(void)(
		riscv_set_register(target, GDB_REGNO_S0, s0) |
		riscv_set_register(target, GDB_REGNO_S1, s1));
	return result;
}

/** @return error code */
static int
riscv_013_read_memory(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint32_t const count,
	uint8_t *const buffer)
{
	riscv_013_info_t *const info = get_info(target);
	assert(info);

	if (info->progbufsize >= 2 && !riscv_prefer_sba)
		return read_memory_progbuf(target, address, size, count, buffer);

	if ((FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS8) && size == 1) ||
		(FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS16) && size == 2) ||
		(FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS32) && size == 4) ||
		(FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS64) && size == 8) ||
		(FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS128) && size == 16)
		) {
		return
			(0 == FIELD_GET(info->sbcs, DMI_SBCS_SBVERSION) ?
				read_memory_bus_v0 :
				read_memory_bus_v1)(target, address, size, count, buffer);
	}

	if (2 <= info->progbufsize)
		return read_memory_progbuf(target, address, size, count, buffer);

	LOG_ERROR("%s: Don't know how to read memory on this target.", target_name(target));
	return ERROR_TARGET_INVALID;
}

/** @return error code */
static int
write_memory_bus_v0(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint32_t const count,
	uint8_t const *const buffer)
{
	/*
	1) write @c sbaddress:
	for @c singlewrite and @c autoincrement, we need to write the address once
	*/
	LOG_DEBUG("%s: System Bus Access: size: %d\tcount:%d\tstart address: 0x%08"
		TARGET_PRIxADDR, target_name(target), size, count, address);

	{
		int const error_code = dmi_write(target, DMI_SBADDRESS0, address);

		if (ERROR_OK != error_code)
			return error_code;
	}

	/* count is in bytes here */
	if (1 == count) {
		/* B.8 Writing Memory, single write check if we write in one go */
		uint8_t const *const t_buffer = buffer;

		int64_t value;

		/* check the size */
		switch (size) {
		case 1:
			value =
				(uint32_t)(t_buffer[0]) << 0 * CHAR_BIT;
			break;

		case 2:
			value =
				(uint32_t)(t_buffer[0]) << 0 * CHAR_BIT |
				(uint32_t)(t_buffer[1]) << 1 * CHAR_BIT;
			break;

		case 4:
			value =
				(uint32_t)(t_buffer[0]) << 0 * CHAR_BIT |
				(uint32_t)(t_buffer[1]) << 1 * CHAR_BIT |
				(uint32_t)(t_buffer[2]) << 2 * CHAR_BIT |
				(uint32_t)(t_buffer[3]) << 3 * CHAR_BIT;
			break;

		default:
			LOG_ERROR("%s: unsupported access size: %d", target_name(target), size);
			return ERROR_TARGET_INVALID;
		}

		int64_t const access = FIELD_SET(0, DMI_SBCS_SBACCESS, size / 2);
		{
			int const error_code = dmi_write(target, DMI_SBCS, access);

			if (ERROR_OK != error_code)
				return error_code;
		}
		LOG_DEBUG("%s: access:  0x%08" PRIx64, target_name(target), access);
		LOG_DEBUG("%s: write_memory:SAB: ONE OFF: value 0x%08" PRIx64,
			target_name(target), value);
		return dmi_write(target, DMI_SBDATA0, value);
	} else {
		/*B.8 Writing Memory, using autoincrement*/

		int64_t const access = FIELD_SET(FIELD_SET(0, DMI_SBCS_SBACCESS, size / 2), DMI_SBCS_SBAUTOINCREMENT, 1);
		LOG_DEBUG("%s: access:  0x%08" PRIx64, target_name(target), access);
		{
			int const error_code = dmi_write(target, DMI_SBCS, access);

			if (ERROR_OK != error_code)
				return error_code;
		}

		/*2)set the value according to the size required and write*/
		for (riscv_addr_t i = 0; i < count; ++i) {
			riscv_addr_t const offset = size * i;
			/* for monitoring only */
			riscv_addr_t const t_addr = address + offset;
			uint8_t const *const t_buffer = buffer + offset;
			int64_t value;

			switch (size) {
			case 1:
				value = t_buffer[0];
				break;

			case 2:
				value =
					(uint32_t)(t_buffer[0]) << 0 * CHAR_BIT |
					(uint32_t)(t_buffer[1]) << 1 * CHAR_BIT;
				break;

			case 4:
				value =
					(uint32_t)(t_buffer[0]) << 0 * CHAR_BIT |
					(uint32_t)(t_buffer[1]) << 1 * CHAR_BIT |
					(uint32_t)(t_buffer[2]) << 2 * CHAR_BIT |
					(uint32_t)(t_buffer[3]) << 3 * CHAR_BIT;
				break;

			default:
				LOG_ERROR("%s: unsupported access size: %d", target_name(target), size);
				return ERROR_TARGET_INVALID;
			}

			LOG_DEBUG("%s: SAB:autoincrement: expected address: 0x%08x value: 0x%08x" PRIx64,
				target_name(target), (uint32_t)(t_addr), (uint32_t)(value));
			{
				int const error_code = dmi_write(target, DMI_SBDATA0, value);

				if (ERROR_OK != error_code)
					return error_code;
			}
		}

		/* reset the auto increment when finished (something weird is happening if this is not done at the end) */
		return dmi_write(target, DMI_SBCS, FIELD_SET(access, DMI_SBCS_SBAUTOINCREMENT, 0));
	}
}

/** @return error code */
static int
write_memory_bus_v1(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint32_t const count,
	uint8_t const *const buffer/**<[out]*/)
{
	riscv_013_info_t *const info = get_info(target);
	uint32_t sbcs = FIELD_SET(sb_sbaccess(size), DMI_SBCS_SBAUTOINCREMENT, 1);
	{
		int const error_code = dmi_write(target, DMI_SBCS, sbcs);

		if (ERROR_OK != error_code)
			return error_code;
	}

	target_addr_t next_address = address;
	target_addr_t const end_address = address + count * size;

	sb_write_address(target, next_address);
	while (next_address < end_address) {
		for (uint32_t i = (next_address - address) / size; i < count; ++i) {
			uint8_t const *const p = buffer + i * size;

			if (3 * sizeof(uint32_t) < size) {
				int const error_code =
					dmi_write(target, DMI_SBDATA3,
					(uint32_t)(p[3 * sizeof(uint32_t) + 0]) << 0 * CHAR_BIT |
						(uint32_t)(p[3 * sizeof(uint32_t) + 1]) << 1 * CHAR_BIT |
						(uint32_t)(p[3 * sizeof(uint32_t) + 2]) << 2 * CHAR_BIT |
						(uint32_t)(p[3 * sizeof(uint32_t) + 3]) << 3 * CHAR_BIT);

				if (ERROR_OK != error_code)
					return error_code;
			}

			if (2 * sizeof(uint32_t) < size) {
				int const error_code =
					dmi_write(target, DMI_SBDATA2,
					(uint32_t)(p[2 * sizeof(uint32_t) + 0]) << 0 * CHAR_BIT |
						(uint32_t)(p[2 * sizeof(uint32_t) + 1]) << 1 * CHAR_BIT |
						(uint32_t)(p[2 * sizeof(uint32_t) + 2]) << 2 * CHAR_BIT |
						(uint32_t)(p[2 * sizeof(uint32_t) + 3]) << 3 * CHAR_BIT);

				if (ERROR_OK != error_code)
					return error_code;
			}

			if (1 * sizeof(uint32_t) < size) {
				int const error_code =
					dmi_write(target, DMI_SBDATA1,
					(uint32_t)(p[1 * sizeof(uint32_t) + 0]) << 0 * CHAR_BIT |
						(uint32_t)(p[1 * sizeof(uint32_t) + 1]) << 1 * CHAR_BIT |
						(uint32_t)(p[1 * sizeof(uint32_t) + 2]) << 2 * CHAR_BIT |
						(uint32_t)(p[1 * sizeof(uint32_t) + 3]) << 3 * CHAR_BIT);

				if (ERROR_OK != error_code)
					return error_code;
			}

			uint32_t value = p[0];

			if (2 < size) {
				value |= (uint32_t)(p[2]) << 2 * CHAR_BIT;
				value |= (uint32_t)(p[3]) << 3 * CHAR_BIT;
			}

			if (1 < size)
				value |= (uint32_t)(p[1]) << 1 * CHAR_BIT;

			assert(0 != size);
			{
				int const error_code = dmi_write(target, DMI_SBDATA0, value);

				if (ERROR_OK != error_code)
					return error_code;
			}

			log_memory_access(address + i * size, value, size, false);

			if (info->bus_master_write_delay) {
				jtag_add_runtest(info->bus_master_write_delay, TAP_IDLE);

				int const error_code = jtag_execute_queue();

				if (ERROR_OK != error_code) {
					LOG_ERROR("%s: Failed to scan idle sequence", target_name(target));
					return error_code;
				}
			}
		}

		{
			int const error_code = read_sbcs_nonbusy(target, &sbcs);

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (FIELD_GET(sbcs, DMI_SBCS_SBBUSYERROR)) {
			/* We wrote while the target was busy. Slow down and try again. */
			{
				int const error_code = dmi_write(target, DMI_SBCS, DMI_SBCS_SBBUSYERROR);

				if (ERROR_OK != error_code)
					return error_code;
			}
			next_address = sb_read_address(target);
			info->bus_master_write_delay += info->bus_master_write_delay / 10 + 1;
			continue;
		}

		unsigned const error = FIELD_GET(sbcs, DMI_SBCS_SBERROR);

		if (0 == error) {
			next_address = end_address;
		} else {
			/* Some error indicating the bus access failed, but not because of
			* something we did wrong. */
			LOG_WARNING("%s: sbcs error %" PRIx32,
				target_name(target), error);
			int const err_code = dmi_write(target, DMI_SBCS, DMI_SBCS_SBERROR);
			(void)(err_code);
			return ERROR_TARGET_FAILURE;
		}
	}

	return ERROR_OK;
}

/** @return error code */
static int
write_memory_progbuf(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint32_t const count,
	uint8_t const *const buffer/**<[in]*/)
{
	assert(target);
	LOG_DEBUG("%s: writing %d words of %d bytes to 0x%08" TARGET_PRIxADDR,
		target_name(target), count, size, address);

	select_dmi(target->tap);

	/* s0 holds the next address to write to */
	uint64_t s0;
	{
		int const error_code = riscv_013_register_read(target, &s0, GDB_REGNO_S0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	/* s1 holds the next data value to write */
	uint64_t s1;
	{
		int const error_code = riscv_013_register_read(target, &s1, GDB_REGNO_S1);

		if (ERROR_OK != error_code)
			return error_code;
	}

	/* Write the program (store, increment) */
	struct riscv_program program;
	riscv_program_init(&program, target);

	int result = ERROR_OK;

	switch (size) {
	case 1:
		riscv_program_sbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
		break;

	case 2:
		riscv_program_shr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
		break;

	case 4:
		riscv_program_swr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
		break;

	default:
		LOG_ERROR("%s: Unsupported size: %d", target_name(target), size);
		result = ERROR_TARGET_INVALID;
		goto error;
	}

	riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);

	result = riscv_program_ebreak(&program);

	if (ERROR_OK != result)
		goto error;

	riscv_program_write(&program);

	riscv_addr_t fin_addr = address + (count * size);
	bool setup_needed = true;
	LOG_DEBUG("%s: writing until final address 0x%016" PRIx64,
		target_name(target), fin_addr);

	riscv_013_info_t *const info = get_info(target);
	assert(info);

	for (riscv_addr_t cur_addr = address; cur_addr < fin_addr;) {
		LOG_DEBUG("%s: transferring burst starting at address 0x%016" PRIx64,
			target_name(target), cur_addr);

		struct riscv_batch *const batch =
			riscv_batch_alloc(target, 32, info->dmi_busy_delay + info->ac_busy_delay);

		/* To write another word, we put it in S1 and execute the program. */
		unsigned const start = (cur_addr - address) / size;

		for (unsigned i = start; i < count; ++i) {
			unsigned const offset = size*i;
			uint8_t const *const t_buffer = buffer + offset;

			uint32_t value;

			switch (size) {
			case 1:
				value = (uint32_t)(t_buffer[0]) << 0 * CHAR_BIT;
				break;

			case 2:
				value =
					(uint32_t)(t_buffer[0]) << 0 * CHAR_BIT |
					(uint32_t)(t_buffer[1]) << 1 * CHAR_BIT;
				break;

			case 4:
				value =
					(uint32_t)(t_buffer[0]) << 0 * CHAR_BIT |
					(uint32_t)(t_buffer[1]) << 1 * CHAR_BIT |
					(uint32_t)(t_buffer[2]) << 2 * CHAR_BIT |
					(uint32_t)(t_buffer[3]) << 3 * CHAR_BIT;
				break;

			default:
				LOG_ERROR("%s: unsupported access size: %d", target_name(target), size);
				riscv_batch_free(batch);
				result = ERROR_TARGET_INVALID;
				goto error;
			}

			log_memory_access(address + offset, value, size, false);
			cur_addr += size;

			if (setup_needed) {
				result = riscv_013_register_write_direct(target, GDB_REGNO_S0,
					address + offset);

				if (ERROR_OK != result) {
					riscv_batch_free(batch);
					goto error;
				}

				/* Write value. */
				result = dmi_write(target, DMI_DATA0, value);

				if (ERROR_OK == result) {
					/* Write and execute command that moves value into S1 and
					* executes program buffer. */
					uint32_t command =
						access_register_command(target,
							GDB_REGNO_S1, 32,
							AC_ACCESS_REGISTER_POSTEXEC |
							AC_ACCESS_REGISTER_TRANSFER |
							AC_ACCESS_REGISTER_WRITE);

					if (ERROR_OK == (result = execute_abstract_command(target, command))) {
						/* Turn on autoexec */
						result =
							dmi_write(target, DMI_ABSTRACTAUTO, 1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);
					}
				}

				if (ERROR_OK != result) {
					riscv_batch_free(batch);
					goto error;
				}

				setup_needed = false;
			} else {
				riscv_batch_add_dmi_write(batch, DMI_DATA0, value);
				if (riscv_batch_full(batch))
					break;
			}
		}

		result = riscv_batch_run(batch);
		riscv_batch_free(batch);

		if (ERROR_OK != result)
			goto error;

		/*	Note that if the scan resulted in a Busy DMI response,
		is this read to abstractcs that will cause the dmi_busy_delay
		to be incremented if necessary.
		*/

		uint32_t abstractcs;

		do {
			int const err = dmi_read(target, &abstractcs, DMI_ABSTRACTCS);

			if (ERROR_OK != err)
				return err;

		} while (0 != FIELD_GET(abstractcs, DMI_ABSTRACTCS_BUSY));

		info->cmderr = FIELD_GET(abstractcs, DMI_ABSTRACTCS_CMDERR);

		switch (info->cmderr) {
		case CMDERR_NONE:
			LOG_DEBUG("%s: successful (partial?) memory write", target_name(target));
			break;

		case CMDERR_BUSY:
			LOG_DEBUG("%s: memory write resulted in busy response", target_name(target));
			riscv_013_clear_abstract_error(target);
			increase_ac_busy_delay(target);

			if (!(
				ERROR_OK == (result = dmi_write(target, DMI_ABSTRACTAUTO, 0)) &&
				ERROR_OK == (result = riscv_013_register_read_direct(target, &cur_addr, GDB_REGNO_S0))
				))
				goto error;
			setup_needed = true;
			break;

		default:
			LOG_ERROR("%s: error when writing memory, abstractcs=0x%08" PRIx32,
				target_name(target), abstractcs);
			riscv_013_clear_abstract_error(target);
			result = ERROR_TARGET_FAILURE;
			goto error;
		}
	}
	goto finish;

error:
	LOG_ERROR("%s: write_memory_progbuf error (%d)",
		target_name(target), result);

finish:
	{
		int const err =
			dmi_write(target, DMI_ABSTRACTAUTO, 0) |
			riscv_013_register_write_direct(target, GDB_REGNO_S1, s1) |
			riscv_013_register_write_direct(target, GDB_REGNO_S0, s0) |
			execute_fence(target);
		(void)(err);
		return result;
	}
}

/** @return error code */
static int
riscv_013_write_memory(struct target *const target,
	target_addr_t address,
	uint32_t const size,
	uint32_t const count,
	uint8_t const *const buffer)
{
	riscv_013_info_t *const info = get_info(target);

	if (info->progbufsize >= 2 && !riscv_prefer_sba)
		return write_memory_progbuf(target, address, size, count, buffer);

	if ((0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS8) && 1 == size) ||
		(0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS16) && 2 == size) ||
		(0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS32) && 4 == size) ||
		(0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS64) && 8 == size) ||
		(0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS128) && 16 == size)
		) {
		switch (FIELD_GET(info->sbcs, DMI_SBCS_SBVERSION)) {
		case 0:
			return write_memory_bus_v0(target, address, size, count, buffer);

		case 1:
			return write_memory_bus_v1(target, address, size, count, buffer);

		default:
			break;
		}
	}

	if (2 <= info->progbufsize)
		return write_memory_progbuf(target, address, size, count, buffer);

	LOG_ERROR("%s: Don't know how to write memory on this target.", target_name(target));
	return ERROR_TARGET_INVALID;
}

static char const *
gdb_regno_name(enum gdb_riscv_regno const regno)
{
	switch (regno) {
	case GDB_REGNO_ZERO:
		return "zero";

	case GDB_REGNO_S0:
		return "s0";

	case GDB_REGNO_S1:
		return "s1";

	case GDB_REGNO_PC:
		return "pc";

	case GDB_REGNO_FPR0:
		return "fpr0";

	case GDB_REGNO_FPR31:
		return "fpr31";

	case GDB_REGNO_CSR0:
		return "csr0";

	case GDB_REGNO_TSELECT:
		return "tselect";

	case GDB_REGNO_TDATA1:
		return "tdata1";

	case GDB_REGNO_TDATA2:
		return "tdata2";

	case GDB_REGNO_MISA:
		return "misa";

	case GDB_REGNO_DPC:
		return "dpc";

	case GDB_REGNO_DCSR:
		return "dcsr";

	case GDB_REGNO_DSCRATCH:
		return "dscratch";

	case GDB_REGNO_MSTATUS:
		return "mstatus";

	case GDB_REGNO_PRIV:
		return "priv";

	default:
		{
			static char buf[32] = {[sizeof buf - 1] = '\0'};

			if (regno <= GDB_REGNO_XPR31)
				snprintf(buf, sizeof buf - 1, "x%d", regno - GDB_REGNO_ZERO);
			else if (GDB_REGNO_CSR0 <= regno && regno <= GDB_REGNO_CSR4095)
				snprintf(buf, sizeof buf - 1, "csr%d", regno - GDB_REGNO_CSR0);
			else if (GDB_REGNO_FPR0 <= regno && regno <= GDB_REGNO_FPR31)
				snprintf(buf, sizeof buf - 1, "f%d", regno - GDB_REGNO_FPR0);
			else
				snprintf(buf, sizeof buf - 1, "gdb_regno_%d", regno);

			return buf;
		}
	}
}

/** @return error code */
static int
__attribute__((warn_unused_result))
riscv_013_get_register(struct target *const target,
	riscv_reg_t *const value,
	int const hid,
	int const rid)
{
	LOG_DEBUG("%s: reading register %s on hart %d",
		target_name(target), gdb_regno_name(rid), hid);

	riscv_set_current_hartid(target, hid);

	int result = ERROR_OK;

	if (GDB_REGNO_PC == rid) {
		result = riscv_013_register_read(target, value, GDB_REGNO_DPC);
		LOG_DEBUG("%s: read PC from DPC: 0x%016" PRIx64,
			target_name(target), *value);
	} else if (GDB_REGNO_PRIV == rid) {
		uint64_t dcsr;
		result = riscv_013_register_read(target, &dcsr, GDB_REGNO_DCSR);
		*value = FIELD_GET(dcsr, CSR_DCSR_PRV);
	} else {
		result = riscv_013_register_read(target, value, rid);
		if (ERROR_OK != result)
			*value = -1;
	}

	return result;
}

/** @return error code */
static int
riscv_013_set_register(struct target *const target, int hid, int rid, uint64_t value)
{
	LOG_DEBUG("%s: writing 0x%" PRIx64 " to register %s on hart %d", target_name(target), value,
		gdb_regno_name(rid), hid);

	riscv_set_current_hartid(target, hid);

	if (rid <= GDB_REGNO_XPR31) {
		return riscv_013_register_write_direct(target, rid, value);
	} else if (rid == GDB_REGNO_PC) {
		LOG_DEBUG("%s: writing PC to DPC: 0x%016" PRIx64, target_name(target), value);
		{
			int const error_code = riscv_013_register_write_direct(target, GDB_REGNO_DPC, value);

			if (ERROR_OK != error_code)
				return error_code;
		}
		uint64_t actual_value;
		{
			int const error_code = riscv_013_register_read_direct(target, &actual_value, GDB_REGNO_DPC);

			if (ERROR_OK != error_code)
				return error_code;
		}
		LOG_DEBUG("%s:   actual DPC written: 0x%016" PRIx64, target_name(target), actual_value);
		if (value != actual_value) {
			LOG_ERROR("%s: Written pc (0x%" PRIx64 ") does not match read back value (0x%" PRIx64 ")",
				target_name(target), value, actual_value);
			return ERROR_TARGET_FAILURE;
		}
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		riscv_013_register_read(target, &dcsr, GDB_REGNO_DCSR);
		dcsr = FIELD_SET(dcsr, CSR_DCSR_PRV, value);
		return riscv_013_register_write_direct(target, GDB_REGNO_DCSR, dcsr);
	} else {
		return riscv_013_register_write_direct(target, rid, value);
	}

	return ERROR_OK;
}

/**	@return error code */
static int
riscv_set_register_on_hart(struct target *const target,
	int const hartid,
	enum gdb_riscv_regno const regid,
	uint64_t const value)
{
	LOG_DEBUG("%s: [%d] %s <- %" PRIx64, target_name(target), hartid, gdb_regno_name(regid), value);
	return riscv_013_set_register(target, hartid, regid, value);
}

/**	@brief Count triggers, and initialize trigger_count for each hart.

trigger_count is initialized even if this function fails to discover something.

Disable any hardware triggers that have @c dmode set.
We can't have set them ourselves.
Maybe they're left over from some killed debug session.

@return error code
*/
static int
riscv_enumerate_triggers(struct target *const target)
{
	riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);

	if (rvi->triggers_enumerated)
		return ERROR_OK;

	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		riscv_reg_t tselect;
		{
			int const error_code =
				riscv_get_register_on_hart(target, &tselect, hartid, GDB_REGNO_TSELECT);

			if (ERROR_OK != error_code)
				return error_code;
		}

		for (unsigned t = 0; t < RISCV_MAX_TRIGGERS; ++t) {
			rvi->harts[hartid].trigger_count = t;

			riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, t);
			uint64_t tselect_rb;
			{
				int const error_code =
					riscv_get_register_on_hart(target, &tselect_rb, hartid, GDB_REGNO_TSELECT);

				if (ERROR_OK != error_code)
					return error_code;
			}

			/* Mask off the top bit, which is used as @c tdrmode in old implementations. */
			tselect_rb &= ~(1ULL << (riscv_xlen(target) - 1));

			if (tselect_rb != t)
				break;

			uint64_t tdata1;
			{
				int const error_code =
					riscv_get_register_on_hart(target, &tdata1, hartid, GDB_REGNO_TDATA1);

				if (ERROR_OK != error_code)
					return error_code;
			}

			int const type = FIELD_GET(tdata1, MCONTROL_TYPE(riscv_xlen(target)));

			switch (type) {
			case 1:
				{
					/* On these older cores we don't support software using triggers. */
					int const error_code = riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);

					if (ERROR_OK != error_code)
						return error_code;
				}
				break;

			case 2:
				if (tdata1 & MCONTROL_DMODE(riscv_xlen(target))) {
					int const error_code = riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);

					if (ERROR_OK != error_code)
						return error_code;
				}
				break;

				/**
				@bug no default case
				*/
			}
		}

		{
			int const error_code =
				riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect);

			if (ERROR_OK != error_code)
				return error_code;
		}

		LOG_INFO("%s: [%d] Found %d triggers",
			target_name(target), hartid, rvi->harts[hartid].trigger_count);
	}

	rvi->triggers_enumerated = true;
	return ERROR_OK;
}

/**
@bug nonhandled errors
*/
static riscv_halt_reason_t
riscv_013_halt_reason(struct target *const target)
{
	riscv_reg_t dcsr;
	{
		int const error_code = riscv_013_register_read(target, &dcsr, GDB_REGNO_DCSR);

		if (ERROR_OK != error_code)
			return RISCV_HALT_UNKNOWN;
	}

	switch (FIELD_GET(dcsr, CSR_DCSR_CAUSE)) {
	case CSR_DCSR_CAUSE_SWBP:
		return RISCV_HALT_BREAKPOINT;

	case CSR_DCSR_CAUSE_TRIGGER:
		/* We could get here before triggers are enumerated if a trigger was
		* already set when we connected. Force enumeration now, which has the
		* side effect of clearing any triggers we did not set. */
		riscv_enumerate_triggers(target);
		return RISCV_HALT_TRIGGER;

	case CSR_DCSR_CAUSE_STEP:
		return RISCV_HALT_SINGLESTEP;

	case CSR_DCSR_CAUSE_DEBUGINT:
	case CSR_DCSR_CAUSE_HALT:
		return RISCV_HALT_INTERRUPT;
	}

	LOG_ERROR("%s: Unknown cause field (%" PRIx64 ") of DCSR (0x%016" PRIx64 ")",
		target_name(target), FIELD_GET(dcsr, CSR_DCSR_CAUSE), dcsr);
	return RISCV_HALT_UNKNOWN;
}

/** @return error code */
int
riscv_write_debug_buffer(struct target *const target,
	unsigned const index,
	uint32_t const data)
{
	return dmi_write(target, DMI_PROGBUF0 + index, data);
}

/**
@bug non handled errors, possible invalid result
*/
uint32_t
riscv_read_debug_buffer(struct target *const target,
	unsigned const index)
{
	uint32_t value;
	{
		int const error_code = dmi_read(target, &value, DMI_PROGBUF0 + index);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: dmi_read return error %d, read value invalid",
				target_name(target), error_code);
			return 0xBADC0DE;
		}
	}

	return value;
}

/** @return error code */
int
riscv_execute_debug_buffer(struct target *const target)
{
	uint32_t const run_program =
		FIELD_SET(
			FIELD_SET(
				FIELD_SET(
					FIELD_SET(0,
						AC_ACCESS_REGISTER_SIZE, 2),
					AC_ACCESS_REGISTER_POSTEXEC, 1),
				AC_ACCESS_REGISTER_TRANSFER, 0),
			AC_ACCESS_REGISTER_REGNO, 0x1000);

	return execute_abstract_command(target, run_program);
}

void
riscv_fill_dmi_write_u64(struct target *const target,
	uint8_t *const buf/**<[out]*/,
	int const a,
	uint64_t const d)
{
	riscv_013_info_t *const info = get_info(target);
	buf_set_u64(buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_WRITE);
	buf_set_u64(buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, d);
	buf_set_u64(buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

void
riscv_fill_dmi_read_u64(struct target *const target,
	uint8_t *const buf/**<[out]*/,
	int a)
{
	riscv_013_info_t *const info = get_info(target);
	buf_set_u64(buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_READ);
	buf_set_u64(buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
	buf_set_u64(buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

void
riscv_fill_dmi_nop_u64(struct target *const target,
	uint8_t *const buf/**<[out]*/)
{
	riscv_013_info_t *const info = get_info(target);
	buf_set_u64(buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_NOP);
	buf_set_u64(buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
	buf_set_u64(buf, DTM_DMI_ADDRESS_OFFSET, info->abits, 0);
}

/* Helper function for riscv_013_test_sba_config_reg */
static int
__attribute__((pure))
get_max_sbaccess(struct target *const target)
{
	riscv_013_info_t const *const info = get_info(target);

	bool const sbaccess8 = ;

	if (0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS128))
		return 4;

	if (0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS64))
		return 3;

	if (0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS32))
		return 2;

	if (0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS16))
		return 1;

	if (0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS8))
		return 0;

	return -1;
}

static uint32_t
__attribute__((pure))
get_num_sbdata_regs(struct target *const target)
{
	riscv_013_info_t const *const info = get_info(target);

	if (0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS128))
		return 4;

	if (0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS64))
		return 2;

	if (0 != FIELD_GET(info->sbcs, DMI_SBCS_SBACCESS32))
		return 1;

	return 0;
}

/**
@bug Check error code!
*/
static void
write_memory_sba_simple(struct target *const target,
	target_addr_t const addr,
	uint32_t const *const write_data,
	uint32_t const write_size,
	uint32_t const sbcs)
{
	riscv_013_info_t *const info = get_info(target);

	uint32_t const sba_size = FIELD_GET(info->sbcs, DMI_SBCS_SBASIZE);

	uint32_t rd_sbcs;
	read_sbcs_nonbusy(target, &rd_sbcs);

	uint32_t sbcs_no_readonaddr = FIELD_SET(sbcs, DMI_SBCS_SBREADONADDR, 0);

	{
		int const error_code = dmi_write(target, DMI_SBCS, sbcs_no_readonaddr);

		if (ERROR_OK != error_code)
			return (void)(error_code);
	}

	assert(32u <= sba_size);

	for (uint32_t i = 0; i < sba_size / 32; ++i) {
		uint32_t const masked_addr = UINT32_MAX & (addr >> 32 * i);
		int const error_code =
			dmi_write(target, i != 3 ? DMI_SBADDRESS0 + i : DMI_SBADDRESS3, masked_addr);

		if (ERROR_OK != error_code)
			return (void)(error_code);
	}

	/*Write SBDATA registers starting with highest address, since write to SBDATA0 triggers write */
	for (int i = write_size - 1; i >= 0; --i) {
		int const error_code = dmi_write(target, DMI_SBDATA0 + i, write_data[i]);

		if (ERROR_OK != error_code)
			return (void)(error_code);
	}
}

/**
@bug non-propagated errors
*/
static void
read_memory_sba_simple(struct target *const target,
	target_addr_t const addr,
	uint32_t *const rd_buf/**<[out]*/,
	uint32_t const read_size,
	uint32_t const sbcs)
{
	riscv_013_info_t *const info = get_info(target);
	assert(info);
	uint32_t const sba_size = FIELD_GET(info->sbcs, DMI_SBCS_SBASIZE);

	uint32_t rd_sbcs;
	read_sbcs_nonbusy(target, &rd_sbcs);

	uint32_t sbcs_readonaddr = FIELD_SET(sbcs, DMI_SBCS_SBREADONADDR, 1);
	{
		int const error_code = dmi_write(target, DMI_SBCS, sbcs_readonaddr);

		if (ERROR_OK != error_code)
			return (void)(error_code);
	}

	/* Write addresses starting with highest address register */
	/**
	@todo assert(0 <= (int)(sba_size / 32 - 1));
	*/
	for (int i = sba_size / 32 - 1; 0 <= i; --i) {
		uint32_t const masked_addr = (addr >> 32 * i) & 0xffffffff;

		int const error_code =
			i != 3 ?
			dmi_write(target, DMI_SBADDRESS0 + i, masked_addr) :
			dmi_write(target, DMI_SBADDRESS3, masked_addr);

		if (ERROR_OK != error_code)
			return (void)(error_code);
	}

	read_sbcs_nonbusy(target, &rd_sbcs);

	for (uint32_t i = 0; i < read_size; ++i) {
		int const error_code = dmi_read(target, &rd_buf[i], DMI_SBDATA0 + i);

		if (ERROR_OK != error_code)
			LOG_WARNING("%s: unhandled dmi_read error, error code %d, read invalid data",
				target_name(target), error_code);
	}
}

/** @return error code */
static int
riscv_013_test_sba_config_reg(struct target *const target,
	target_addr_t const legal_address,
	uint32_t const num_words,
	target_addr_t const illegal_address,
	bool const run_sbbusyerror_test)
{
	LOG_INFO("%s: Testing System Bus Access as defined by RISC-V Debug Spec v0.13", target_name(target));

	uint32_t sbcs_orig;
	{
		/**
		@todo Handle error
		*/
		int const error_code = dmi_read(target, &sbcs_orig, DMI_SBCS);

		if (ERROR_OK != error_code)
			LOG_WARNING("%s: dmi_read return error code %d",
				target_name(target), error_code);
	}

	uint32_t sbcs = sbcs_orig;

	int const max_sbaccess = get_max_sbaccess(target);

	if (max_sbaccess < 0) {
		LOG_ERROR("%s: System Bus Access not supported in this config.", target_name(target));
		return ERROR_TARGET_INVALID;
	}

	if (FIELD_GET(sbcs, DMI_SBCS_SBVERSION) != 1) {
		LOG_ERROR("%s: System Bus Access unsupported SBVERSION (%d). Only version 1 is supported.",
			target_name(target), FIELD_GET(sbcs, DMI_SBCS_SBVERSION));
		return ERROR_TARGET_FAILURE;
	}

	uint32_t num_sbdata_regs = get_num_sbdata_regs(target);

	/**
	@bug Non-portable language extension: array with dynamic size
	*/
	uint32_t rd_buf[num_sbdata_regs];

	/* Test 1: Simple write/read test */
	bool test_passed = true;
	sbcs = FIELD_SET(sbcs_orig, DMI_SBCS_SBAUTOINCREMENT, 0);
	{
		int const error_code = dmi_write(target, DMI_SBCS, sbcs);

		if (ERROR_OK != error_code)
			return error_code;
	}

	static uint32_t const test_patterns[4] = {0xdeadbeef, 0xfeedbabe, 0x12345678, 0x08675309};
	unsigned tests_failed = 0u;

	for (uint32_t sbaccess = 0; sbaccess <= (uint32_t)(max_sbaccess); ++sbaccess) {
		sbcs = FIELD_SET(sbcs, DMI_SBCS_SBACCESS, sbaccess);
		{
			int const error_code = dmi_write(target, DMI_SBCS, sbcs);

			if (ERROR_OK != error_code)
				return error_code;
		}

		uint32_t compare_mask =
			sbaccess == 0 ? UINT32_C(0xff) :
			sbaccess == 1 ? UINT32_C(0xffff) :
			UINT32_C(0xffffffff);

		for (uint32_t i = 0; i < num_words; ++i) {
			uint32_t addr = legal_address + (i << sbaccess);
			/**
			@bug Non-portable language extension: array with dynamic size
			*/
			uint32_t wr_data[num_sbdata_regs];

			for (uint32_t j = 0; j < num_sbdata_regs; ++j)
				wr_data[j] = test_patterns[j] + i;

			write_memory_sba_simple(target, addr, wr_data, num_sbdata_regs, sbcs);
		}

		for (uint32_t i = 0; i < num_words; ++i) {
			uint32_t const addr = legal_address + (i << sbaccess);
			read_memory_sba_simple(target, addr, rd_buf, num_sbdata_regs, sbcs);
			for (uint32_t j = 0; j < num_sbdata_regs; ++j) {

				if ((compare_mask & (test_patterns[j] + i)) != (compare_mask & rd_buf[j])) {
					LOG_ERROR("%s:"
						" System Bus Access Test 1:"
						" Error reading non-autoincremented address %" PRIx32
						", expected val = %" PRIx32
						", read val = %" PRIx32,
						target_name(target), addr, test_patterns[j] + i, rd_buf[j]);
					test_passed = false;
					++tests_failed;
				}
			}
		}
	}

	if (test_passed)
		LOG_INFO("%s: System Bus Access Test 1: Simple write/read test PASSED.",
			target_name(target));

	/* Test 2: Address auto increment test */
	target_addr_t curr_addr;
	target_addr_t prev_addr;
	test_passed = true;
	sbcs = FIELD_SET(sbcs_orig, DMI_SBCS_SBAUTOINCREMENT, 1);

	{
		int const error_code = dmi_write(target, DMI_SBCS, sbcs);

		if (ERROR_OK != error_code)
			return error_code;
	}

	for (uint32_t sbaccess = 0; sbaccess <= (uint32_t)(max_sbaccess); ++sbaccess) {
		sbcs = FIELD_SET(sbcs, DMI_SBCS_SBACCESS, sbaccess);
		{
			int const error_code = dmi_write(target, DMI_SBCS, sbcs);

			if (ERROR_OK != error_code)
				return error_code;
		}

		{
			int const error_code = dmi_write(target, DMI_SBADDRESS0, legal_address);

			if (ERROR_OK != error_code)
				return error_code;
		}

		read_sbcs_nonbusy(target, &sbcs);
		curr_addr = legal_address;
		for (uint32_t i = 0; i < num_words; ++i) {
			prev_addr = curr_addr;
			read_sbcs_nonbusy(target, &sbcs);
			curr_addr = sb_read_address(target);
			if ((curr_addr - prev_addr != (uint32_t)(1 << sbaccess)) && (i != 0)) {
				LOG_ERROR("%s: System Bus Access Test 2: Error with address auto-increment, sbaccess = %x.",
					target_name(target), sbaccess);
				test_passed = false;
				++tests_failed;
			}

			{
				int const error_code = dmi_write(target, DMI_SBDATA0, i);

				if (ERROR_OK != error_code)
					return error_code;
			}
		}

		read_sbcs_nonbusy(target, &sbcs);

		{
			int const error_code = dmi_write(target, DMI_SBADDRESS0, legal_address);

			if (ERROR_OK != error_code)
				return error_code;
		}

		uint32_t val;
		sbcs = FIELD_SET(sbcs, DMI_SBCS_SBREADONDATA, 1);
		{
			int const error_code = dmi_write(target, DMI_SBCS, sbcs);

			if (ERROR_OK != error_code)
				return error_code;
		}

		{
			/* Dummy read to trigger first system bus read */
			/**
			@todo Add error handling
			*/
			int const error_code = dmi_read(target, &val, DMI_SBDATA0);

			if (ERROR_OK != error_code)
				LOG_ERROR("%s: dmi_read return status %d",
					target_name(target), error_code);
		}

		curr_addr = legal_address;
		for (uint32_t i = 0; i < num_words; ++i) {
			prev_addr = curr_addr;
			read_sbcs_nonbusy(target, &sbcs);
			curr_addr = sb_read_address(target);

			if (curr_addr != prev_addr + (UINT32_C(1) << sbaccess) &&
				i != 0
				) {
				LOG_ERROR("%s: System Bus Access Test 2: Error with address auto-increment, sbaccess = %x",
					target_name(target), sbaccess);
				test_passed = false;
				++tests_failed;
			}

			if (
				!(
					ERROR_OK == dmi_read(target, &val, DMI_SBDATA0) &&
					ERROR_OK == read_sbcs_nonbusy(target, &sbcs)
					) ||
				i != val
				) {
				LOG_ERROR("%s: System Bus Access Test 2:"
					" Error reading auto-incremented address,"
					" expected val = %" PRIx32 ", read val = %" PRIx32,
					target_name(target), i, val);
				test_passed = false;
				++tests_failed;
			}
		}
	}

	if (test_passed)
		LOG_INFO("%s: System Bus Access Test 2: Address auto-increment test PASSED.", target_name(target));

	/* Test 3: Read from illegal address */
	read_memory_sba_simple(target, illegal_address, rd_buf, 1, sbcs_orig);

	uint32_t rd_val;
	if (ERROR_OK == dmi_read(target, &rd_val, DMI_SBCS) &&
		2 == FIELD_GET(rd_val, DMI_SBCS_SBERROR)
		) {
		sbcs = FIELD_SET(sbcs_orig, DMI_SBCS_SBERROR, 2);

		if (
			ERROR_OK == dmi_write(target, DMI_SBCS, sbcs) &&
			ERROR_OK == dmi_read(target, &rd_val, DMI_SBCS) &&
			0 == FIELD_GET(rd_val, DMI_SBCS_SBERROR)
			)
			LOG_INFO("%s: System Bus Access Test 3: Illegal address read test PASSED.", target_name(target));
		else
			LOG_ERROR("%s: System Bus Access Test 3: Illegal address read test FAILED, unable to clear to 0.",
				target_name(target));
	} else {
		LOG_ERROR("%s: System Bus Access Test 3: Illegal address read test FAILED, unable to set error code.",
			target_name(target));
	}

	/* Test 4: Write to illegal address */
	write_memory_sba_simple(target, illegal_address, test_patterns, 1, sbcs_orig);

	if (
		ERROR_OK == dmi_read(target, &rd_val, DMI_SBCS) &&
		2 == FIELD_GET(rd_val, DMI_SBCS_SBERROR)
		) {
		sbcs = FIELD_SET(sbcs_orig, DMI_SBCS_SBERROR, 2);
		if (
			ERROR_OK == dmi_write(target, DMI_SBCS, sbcs) &&
			ERROR_OK == dmi_read(target, &rd_val, DMI_SBCS) &&
			0 == FIELD_GET(rd_val, DMI_SBCS_SBERROR)
			)
			LOG_INFO("%s: System Bus Access Test 4: Illegal address write test PASSED.", target_name(target));
		else {
			LOG_ERROR("%s: System Bus Access Test 4: Illegal address write test FAILED, unable to clear to 0.",
				target_name(target));
			++tests_failed;
		}
	} else {
		LOG_ERROR("%s: System Bus Access Test 4: Illegal address write test FAILED, unable to set error code.",
			target_name(target));
		++tests_failed;
	}

	/* Test 5: Write with unsupported sbaccess size */
	uint32_t sbaccess128 = FIELD_GET(sbcs_orig, DMI_SBCS_SBACCESS128);

	if (sbaccess128) {
		LOG_INFO("%s: System Bus Access Test 5: SBCS sbaccess error test PASSED, all sbaccess sizes supported.",
			target_name(target));
	} else {
		sbcs = FIELD_SET(sbcs_orig, DMI_SBCS_SBACCESS, 4);

		write_memory_sba_simple(target, legal_address, test_patterns, 1, sbcs);

		if (
			ERROR_OK == dmi_read(target, &rd_val, DMI_SBCS) &&
			4 == FIELD_GET(rd_val, DMI_SBCS_SBERROR)
			) {
			sbcs = FIELD_SET(sbcs_orig, DMI_SBCS_SBERROR, 4);
			if (
				ERROR_OK == dmi_write(target, DMI_SBCS, sbcs) &&
				ERROR_OK == dmi_read(target, &rd_val, DMI_SBCS) &&
				0 == FIELD_GET(rd_val, DMI_SBCS_SBERROR)
				)
				LOG_INFO("%s: System Bus Access Test 5: SBCS sbaccess error test PASSED.", target_name(target));
			else {
				LOG_ERROR("%s: System Bus Access Test 5: SBCS sbaccess error test FAILED, unable to clear to 0.",
					target_name(target));
				++tests_failed;
			}
		} else {
			LOG_ERROR("%s: System Bus Access Test 5: SBCS sbaccess error test FAILED, unable to set error code.",
				target_name(target));
			++tests_failed;
		}
	}

	/* Test 6: Write to misaligned address */
	sbcs = FIELD_SET(sbcs_orig, DMI_SBCS_SBACCESS, 1);

	write_memory_sba_simple(target, legal_address + 1, test_patterns, 1, sbcs);

	if (
		ERROR_OK == dmi_read(target, &rd_val, DMI_SBCS) &&
		FIELD_GET(rd_val, DMI_SBCS_SBERROR) == 3
		) {
		sbcs = FIELD_SET(sbcs_orig, DMI_SBCS_SBERROR, 3);
		if (
			ERROR_OK == dmi_write(target, DMI_SBCS, sbcs) &&
			ERROR_OK == dmi_read(target, &rd_val, DMI_SBCS) &&
			0 == FIELD_GET(rd_val, DMI_SBCS_SBERROR)
			)
			LOG_INFO("%s: System Bus Access Test 6: SBCS address alignment error test PASSED", target_name(target));
		else {
			LOG_ERROR("%s: System Bus Access Test 6: SBCS address alignment error test FAILED, unable to clear to 0.",
				target_name(target));
			++tests_failed;
		}
	} else {
		LOG_ERROR("%s: System Bus Access Test 6: SBCS address alignment error test FAILED, unable to set error code.",
			target_name(target));
		++tests_failed;
	}

	/* Test 7: Set sbbusyerror, only run this case in simulation as it is likely
	* impossible to hit otherwise */
	if (run_sbbusyerror_test) {
		sbcs = FIELD_SET(sbcs_orig, DMI_SBCS_SBREADONADDR, 1);
		{
			int const error_code = dmi_write(target, DMI_SBCS, sbcs);

			if (ERROR_OK != error_code)
				return error_code;
		}

		for (int i = 0; i < 16; ++i) {
			int const error_code = dmi_write(target, DMI_SBDATA0, 0xdeadbeef);

			if (ERROR_OK != error_code)
				return error_code;
		}

		for (int i = 0; i < 16; ++i) {
			int const error_code = dmi_write(target, DMI_SBADDRESS0, legal_address);

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (
			ERROR_OK == dmi_read(target, &rd_val, DMI_SBCS) &&
			0 != FIELD_GET(rd_val, DMI_SBCS_SBBUSYERROR)
			) {

			sbcs = FIELD_SET(sbcs_orig, DMI_SBCS_SBBUSYERROR, 1);

			if (
				ERROR_OK == dmi_write(target, DMI_SBCS, sbcs) &&
				ERROR_OK == dmi_read(target, &rd_val, DMI_SBCS) &&
				0 == FIELD_GET(rd_val, DMI_SBCS_SBBUSYERROR)
				)
				LOG_INFO("%s: System Bus Access Test 7: SBCS sbbusyerror test PASSED.", target_name(target));
			else {
				LOG_ERROR("%s: System Bus Access Test 7: SBCS sbbusyerror test FAILED, unable to clear to 0.",
					target_name(target));
				++tests_failed;
			}
		} else {
			LOG_ERROR("%s: System Bus Access Test 7: SBCS sbbusyerror test FAILED, unable to set error code.",
				target_name(target));
			++tests_failed;
		}
	}

	if (tests_failed == 0) {
		LOG_INFO("%s: ALL TESTS PASSED", target_name(target));
		return ERROR_OK;
	} else {
		LOG_ERROR("%s: %d TESTS FAILED", target_name(target), tests_failed);
		return ERROR_TARGET_FAILURE;
	}
}

int
riscv_dmi_write_u64_bits(struct target *const target)
{
	riscv_013_info_t *const info = get_info(target);
	return info->abits + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH;
}

/** @return error code */
static int
maybe_execute_fence_i(struct target *const target)
{
	riscv_013_info_t *const info = get_info(target);
	riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);

	if (info->progbufsize + rvi->impebreak >= 3)
		return execute_fence(target);

	return ERROR_OK;
}

/** @return error code */
static int
riscv_013_on_step_or_resume(struct target *const target,
	bool const step)
{
	{
		int const error_code = maybe_execute_fence_i(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	/* We want to twiddle some bits in the debug CSR so debugging works. */
	riscv_reg_t dcsr;
	{
		int const error_code = riscv_013_register_read(target, &dcsr, GDB_REGNO_DCSR);

		if (ERROR_OK != error_code)
			return error_code;
	}

	dcsr = FIELD_SET(dcsr, CSR_DCSR_STEP, step);
	dcsr = FIELD_SET(dcsr, CSR_DCSR_EBREAKM, 1);
	dcsr = FIELD_SET(dcsr, CSR_DCSR_EBREAKS, 1);
	dcsr = FIELD_SET(dcsr, CSR_DCSR_EBREAKU, 1);
	return riscv_set_register(target, GDB_REGNO_DCSR, dcsr);
}

/** @return error code */
static int
riscv_013_on_step(struct target *const target)
{
	return riscv_013_on_step_or_resume(target, true);
}

/** @return error code */
static int
riscv_013_on_resume(struct target *const target)
{
	return riscv_013_on_step_or_resume(target, false);
}

/** @return error code */
static int
riscv_013_step_or_resume_current_hart(struct target *const target,
	bool const step)
{
	riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);
	LOG_DEBUG("%s: resuming hart %d (for step?=%d)",
		target_name(target), rvi->current_hartid, step);

	if (!riscv_is_halted(target)) {
		LOG_ERROR("%s: Hart %d is not halted!", target_name(target), rvi->current_hartid);
		return ERROR_TARGET_NOT_HALTED;
	}

	{
		int const error_code = maybe_execute_fence_i(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	/* Issue the resume command, and then wait for the current hart to resume. */
	uint32_t dmcontrol =
		set_hartsel(DMI_DMCONTROL_DMACTIVE, rvi->current_hartid);
	{
		int const error_code =
			dmi_write(target, DMI_DMCONTROL, dmcontrol | DMI_DMCONTROL_RESUMEREQ);

		if (ERROR_OK != error_code)
			return error_code;
	}

	uint32_t dmstatus;

	for (size_t i = 0; i < 256; ++i) {
		usleep(10);

		{
			int const error_code = dmstatus_read(target, &dmstatus, true);

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (
			0 == FIELD_GET(dmstatus, DMI_DMSTATUS_ALLRESUMEACK) ||
			(step && 0 == FIELD_GET(dmstatus, DMI_DMSTATUS_ALLHALTED))
			)
			continue;

		return dmi_write(target, DMI_DMCONTROL, dmcontrol);
	}

	LOG_ERROR("%s: unable to resume hart %d", target_name(target), rvi->current_hartid);

	{
		int const error_code = dmi_read(target, &dmcontrol, DMI_DMCONTROL);

		if (ERROR_OK != error_code)
			return error_code;
	}

	LOG_ERROR("%s:   dmcontrol=0x%08x", target_name(target), dmcontrol);

	{
		int const error_code = dmstatus_read(target, &dmstatus, true);

		if (ERROR_OK != error_code)
			return error_code;
	}

	LOG_ERROR("%s:   dmstatus =0x%08x", target_name(target), dmstatus);

	if (step) {
		LOG_ERROR("%s:   was stepping, halting", target_name(target));
		return riscv_013_halt_current_hart(target);
	}

	return ERROR_TARGET_FAILURE;
}

/** @return error code */
static int
riscv_013_step_current_hart(struct target *const target)
{
	return riscv_013_step_or_resume_current_hart(target, true);
}

/** @return error code */
static int
riscv_013_resume_current_hart(struct target *const target)
{
	return riscv_013_step_or_resume_current_hart(target, false);
}

static int
riscv_halt_one_hart(struct target *const target, int const hartid)
{
	LOG_DEBUG("%s: halting hart %d", target_name(target), hartid);

	{
		int const error_code = riscv_set_current_hartid(target, hartid);

		if (ERROR_OK != error_code)
			return error_code;
	}

	if (riscv_is_halted(target)) {
		LOG_DEBUG("%s:  hart %d requested halt, but was already halted",
			target_name(target), hartid);
		return ERROR_OK;
	}

	return riscv_013_halt_current_hart(target);
}

/**	Run control, possibly for multiple harts.

The _all_harts versions resume all the enabled harts,
which when running in RTOS mode is all the harts on the system.

@return error code
*/
static int
riscv_halt_all_harts(struct target *const target)
{
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (!riscv_hart_enabled(target, i))
			continue;

		riscv_halt_one_hart(target, i);
	}

	riscv_invalidate_register_cache(target);

	return ERROR_OK;
}

/** @return error code*/
static int
riscv_resume_one_hart(struct target *const target, int const hartid)
{
	LOG_DEBUG("%s: resuming hart %d", target_name(target), hartid);

	{
		int const error_code = riscv_set_current_hartid(target, hartid);

		if (ERROR_OK != error_code)
			return error_code;
	}

	if (!riscv_is_halted(target)) {
		LOG_DEBUG("%s:  hart %d requested resume, but was already resumed", target_name(target), hartid);
		return ERROR_OK;
	}

	{
		int const error_code = riscv_013_on_resume(target);

		if (ERROR_OK != error_code)
			return error_code;
	}
	return riscv_013_resume_current_hart(target);
}

/**	Run control, possibly for multiple harts.

The _all_harts versions resume all the enabled harts,
which when running in RTOS mode is all the harts on the system.

@return error code
*/
static int
riscv_resume_all_harts(struct target *const target)
{
	int const number_of_harts = riscv_count_harts(target);
	int error_code = ERROR_OK;

	for (int i = 0; i < number_of_harts; ++i)
		if (riscv_hart_enabled(target, i)) {
			int const error_code_1 = riscv_resume_one_hart(target, i);

			if (ERROR_OK == error_code && ERROR_OK != error_code_1)
				error_code = error_code_1;
		}

	riscv_invalidate_register_cache(target);
	return error_code;
}

static riscv_halt_reason_t
	riscv_halt_reason(struct target *const target,
		int const hartid)
{
	if (ERROR_OK != riscv_set_current_hartid(target, hartid))
		return RISCV_HALT_ERROR;

	if (!riscv_is_halted(target)) {
		LOG_ERROR("%s: Hart is not halted!", target_name(target));
		return RISCV_HALT_UNKNOWN;
	}

	return riscv_013_halt_reason(target);
}

/** @return error code */
static int
riscv_013_test_compliance(struct target *const target)
{
	LOG_INFO("%s: Testing Compliance against RISC-V Debug Spec v0.13", target_name(target));

	if (!riscv_rtos_enabled(target)) {
		LOG_ERROR("%s: Please run with -rtos riscv to run compliance test.", target_name(target));
		return ERROR_TARGET_INVALID;
	}

	int total_tests = 0;
	int passed_tests = 0;

	uint32_t const dmcontrol_orig = DMI_DMCONTROL_DMACTIVE;
	uint32_t dmcontrol;
	uint32_t testvar;
	uint32_t testvar_read;
	riscv_reg_t value;
	riscv_013_info_t *const info = get_info(target);

	/* All the bits of HARTSEL are covered by the examine sequence. */

	/* hartreset */
	/* This field is optional. Either we can read and write it to 1/0,
	or it is tied to 0. This check doesn't really do anything, but
	it does attempt to set the bit to 1 and then back to 0, which needs to
	work if its implemented. */
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, FIELD_SET(dmcontrol_orig, DMI_DMCONTROL_HARTRESET, 1));
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, FIELD_SET(dmcontrol_orig, DMI_DMCONTROL_HARTRESET, 0));
	COMPLIANCE_READ(target, &dmcontrol, DMI_DMCONTROL);
	COMPLIANCE_TEST((FIELD_GET(dmcontrol, DMI_DMCONTROL_HARTRESET) == 0),
		"DMCONTROL.hartreset can be 0 or RW.");

	/* hasel */
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, FIELD_SET(dmcontrol_orig, DMI_DMCONTROL_HASEL, 1));
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, FIELD_SET(dmcontrol_orig, DMI_DMCONTROL_HASEL, 0));
	COMPLIANCE_READ(target, &dmcontrol, DMI_DMCONTROL);
	COMPLIANCE_TEST((FIELD_GET(dmcontrol, DMI_DMCONTROL_HASEL) == 0),
		"DMCONTROL.hasel can be 0 or RW.");
	/**
	@todo test that hamask registers exist if hasel does.
	*/

	/* haltreq */
	COMPLIANCE_MUST_PASS(riscv_halt_all_harts(target));
	/* This bit is not actually readable according to the spec, so nothing to check.*/

	/* DMSTATUS */
	COMPLIANCE_CHECK_RO(target, DMI_DMSTATUS);

	/* resumereq */
	/* This bit is not actually readable according to the spec, so nothing to check.*/
	COMPLIANCE_MUST_PASS(riscv_resume_all_harts(target));

	/* Halt all harts again so the test can continue.*/
	COMPLIANCE_MUST_PASS(riscv_halt_all_harts(target));

	/* HARTINFO: Read-Only. This is per-hart, so need to adjust hartsel. */
	uint32_t hartinfo;
	COMPLIANCE_READ(target, &hartinfo, DMI_HARTINFO);
	for (int hartsel = 0; hartsel < riscv_count_harts(target); ++hartsel) {
		COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, hartsel));

		COMPLIANCE_CHECK_RO(target, DMI_HARTINFO);

		/* $dscratch CSRs */
		uint32_t nscratch = FIELD_GET(hartinfo, DMI_HARTINFO_NSCRATCH);

		for (unsigned d = 0; d < nscratch; ++d) {
			riscv_reg_t testval, testval_read;
			/* Because DSCRATCH is not guaranteed to last across PB executions, need to put
			this all into one PB execution. Which may not be possible on all implementations.*/
			if (info->progbufsize >= 5) {
				for (testval = 0x0011223300112233;
					testval != 0xDEAD;
					testval = testval == 0x0011223300112233 ? ~testval : 0xDEAD) {
					COMPLIANCE_TEST(ERROR_OK == riscv_013_register_write_direct(target, GDB_REGNO_S0, testval),
						"Need to be able to write S0 in order to test DSCRATCH.");
					struct riscv_program program32;
					riscv_program_init(&program32, target);
					riscv_program_csrw(&program32, GDB_REGNO_S0, GDB_REGNO_DSCRATCH + d);
					riscv_program_csrr(&program32, GDB_REGNO_S1, GDB_REGNO_DSCRATCH + d);
					riscv_program_fence(&program32);
					riscv_program_ebreak(&program32);
					COMPLIANCE_TEST(ERROR_OK == riscv_program_exec(&program32, target),
						"Accessing DSCRATCH with program buffer should succeed.");
					COMPLIANCE_TEST(ERROR_OK == riscv_013_register_read_direct(target, &testval_read, GDB_REGNO_S1),
						"Need to be able to read S1 in order to test DSCRATCH.");
					if (riscv_xlen(target) > 32) {
						COMPLIANCE_TEST(testval == testval_read,
							"All DSCRATCH registers in HARTINFO must be R/W.");
					} else {
						COMPLIANCE_TEST(testval_read == (testval & 0xFFFFFFFF),
							"All DSCRATCH registers in HARTINFO must be R/W.");
					}
				}
			}
		}
		/**
		@todo dataaccess
		*/
		if (FIELD_GET(hartinfo, DMI_HARTINFO_DATAACCESS)) {
			/**
			@todo Shadowed in memory map

			@todo datasize

			@todo dataaddr
			*/
		} else {
			/**
			@todo Shadowed in CSRs.

			@todo datasize

			@todo dataaddr
			*/
		}

	}

	/**
	@todo HALTSUM More than 32 harts. Would need to loop over this to set hartsel

	@todo HALTSUM2, HALTSUM3
	*/
	/* HALTSUM0 */
	uint32_t expected_haltsum0 = 0;

	for (int i = 0; i < MIN(riscv_count_harts(target), 32); ++i)
		expected_haltsum0 |= (1 << i);

	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM0);
	COMPLIANCE_TEST(testvar_read == expected_haltsum0,
		"HALTSUM0 should report summary of up to 32 halted harts");

	COMPLIANCE_WRITE(target, DMI_HALTSUM0, 0xffffffff);
	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM0);
	COMPLIANCE_TEST(testvar_read == expected_haltsum0, "HALTSUM0 should be R/O");

	COMPLIANCE_WRITE(target, DMI_HALTSUM0, 0x0);
	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM0);
	COMPLIANCE_TEST(testvar_read == expected_haltsum0, "HALTSUM0 should be R/O");

	/* HALTSUM1 */
	uint32_t expected_haltsum1 = 0;

	for (int i = 0; i < MIN(riscv_count_harts(target), 1024); i += 32)
		expected_haltsum1 |= (1 << (i / 32));

	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM1);
	COMPLIANCE_TEST(testvar_read == expected_haltsum1,
		"HALTSUM1 should report summary of up to 1024 halted harts");

	COMPLIANCE_WRITE(target, DMI_HALTSUM1, 0xffffffff);
	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM1);
	COMPLIANCE_TEST(testvar_read == expected_haltsum1, "HALTSUM1 should be R/O");

	COMPLIANCE_WRITE(target, DMI_HALTSUM1, 0x0);
	COMPLIANCE_READ(target, &testvar_read, DMI_HALTSUM1);
	COMPLIANCE_TEST(testvar_read == expected_haltsum1, "HALTSUM1 should be R/O");

	/**
	@todo HAWINDOWSEL

	@todo HAWINDOW
	*/

	/* ABSTRACTCS */

	uint32_t abstractcs;
	COMPLIANCE_READ(target, &abstractcs, DMI_ABSTRACTCS);

	/* Check that all reported Data Words are really R/W */
	for (int invert = 0; invert < 2; ++invert) {
		for (unsigned i = 0; i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_DATACOUNT); ++i) {
			testvar = (i + 1) * 0x11111111;
			if (invert)
				testvar = ~testvar;
			COMPLIANCE_WRITE(target, DMI_DATA0 + i, testvar);
		}

		for (unsigned i = 0; i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_DATACOUNT); ++i) {
			testvar = (i + 1) * 0x11111111;
			if (invert)
				testvar = ~testvar;
			COMPLIANCE_READ(target, &testvar_read, DMI_DATA0 + i);
			COMPLIANCE_TEST(testvar_read == testvar, "All reported DATA words must be R/W");
		}
	}

	/* Check that all reported ProgBuf words are really R/W */
	for (int invert = 0; invert < 2; ++invert) {
		for (unsigned i = 0; i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE); ++i) {
			testvar = (i + 1) * 0x11111111;

			if (invert)
				testvar = ~testvar;

			COMPLIANCE_WRITE(target, DMI_PROGBUF0 + i, testvar);
		}

		for (unsigned i = 0; i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE); ++i) {
			testvar = (i + 1) * 0x11111111;

			if (invert)
				testvar = ~testvar;

			COMPLIANCE_READ(target, &testvar_read, DMI_PROGBUF0 + i);
			COMPLIANCE_TEST(testvar_read == testvar, "All reported PROGBUF words must be R/W");
		}
	}

	/**
	@todo Cause and clear all error types
	*/

	/* COMMAND
	According to the spec, this register is only W, so can't really check the read result.
	But at any rate, this is not legal and should cause an error. */
	COMPLIANCE_WRITE(target, DMI_COMMAND, 0xAAAAAAAA);
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
	COMPLIANCE_TEST(FIELD_GET(testvar_read,
		DMI_ABSTRACTCS_CMDERR) == CMDERR_NOT_SUPPORTED,
		"Illegal COMMAND should result in UNSUPPORTED");
	COMPLIANCE_WRITE(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);

	COMPLIANCE_WRITE(target, DMI_COMMAND, 0x55555555);
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
	COMPLIANCE_TEST(FIELD_GET(testvar_read,
		DMI_ABSTRACTCS_CMDERR) == CMDERR_NOT_SUPPORTED,
		"Illegal COMMAND should result in UNSUPPORTED");
	COMPLIANCE_WRITE(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);

	/* Basic Abstract Commands */
	for (unsigned i = 1; i < 32; i = i << 1) {
		riscv_reg_t testval = i | ((i + 1ULL) << 32);
		riscv_reg_t testval_read;
		COMPLIANCE_TEST(ERROR_OK == riscv_013_register_write_direct(target, GDB_REGNO_ZERO + i, testval),
			"GPR Writes should be supported.");
		COMPLIANCE_MUST_PASS(write_abstract_arg(target, 0, 0xDEADBEEFDEADBEEF, 64));
		COMPLIANCE_TEST(ERROR_OK == riscv_013_register_read_direct(target, &testval_read, GDB_REGNO_ZERO + i),
			"GPR Reads should be supported.");
		if (riscv_xlen(target) > 32) {
			/* Dummy comment to satisfy linter, since removing the brances here doesn't actually compile. */
			COMPLIANCE_TEST(testval == testval_read, "GPR Reads and writes should be supported.");
		} else {
			/* Dummy comment to satisfy linter, since removing the brances here doesn't actually compile. */
			COMPLIANCE_TEST((testval & 0xFFFFFFFF) == testval_read, "GPR Reads and writes should be supported.");
		}
	}

	/* ABSTRACTAUTO
	See which bits are actually writable */
	COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0xFFFFFFFF);
	uint32_t abstractauto;
	uint32_t busy;
	COMPLIANCE_READ(target, &abstractauto, DMI_ABSTRACTAUTO);
	COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0x0);
	if (abstractauto > 0) {
		/* This mechanism only works when you have a reasonable sized progbuf, which is not
		a true compliance requirement. */
		if (info->progbufsize >= 3) {

			testvar = 0;
			COMPLIANCE_TEST(ERROR_OK == riscv_013_register_write_direct(target, GDB_REGNO_S0, 0),
				"Need to be able to write S0 to test ABSTRACTAUTO");
			struct riscv_program program;
			riscv_program_init(&program, target);
			/* This is also testing that WFI() is a NOP during debug mode. */
			COMPLIANCE_MUST_PASS(riscv_program_insert(&program, wfi()));
			COMPLIANCE_MUST_PASS(riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, 1));
			COMPLIANCE_MUST_PASS(riscv_program_ebreak(&program));
			COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0x0);
			COMPLIANCE_MUST_PASS(riscv_program_exec(&program, target));
			++testvar;
			COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0xFFFFFFFF);
			COMPLIANCE_READ(target, &abstractauto, DMI_ABSTRACTAUTO);
			uint32_t autoexec_data = FIELD_GET(abstractauto, DMI_ABSTRACTAUTO_AUTOEXECDATA);
			uint32_t autoexec_progbuf = FIELD_GET(abstractauto, DMI_ABSTRACTAUTO_AUTOEXECPROGBUF);

			for (unsigned i = 0; i < 12; ++i) {
				COMPLIANCE_READ(target, &testvar_read, DMI_DATA0 + i);
				do {
					COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
					busy = FIELD_GET(testvar_read, DMI_ABSTRACTCS_BUSY);
				} while (busy);
				if (autoexec_data & (1 << i)) {
					COMPLIANCE_TEST(i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_DATACOUNT),
						"AUTOEXEC may be writable up to DATACOUNT bits.");
					++testvar;
				}
			}

			for (unsigned i = 0; i < 16; ++i) {
				COMPLIANCE_READ(target, &testvar_read, DMI_PROGBUF0 + i);
				do {
					COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
					busy = FIELD_GET(testvar_read, DMI_ABSTRACTCS_BUSY);
				} while (busy);
				if (autoexec_progbuf & (1 << i)) {
					COMPLIANCE_TEST(i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE),
						"AUTOEXEC may be writable up to PROGBUFSIZE bits.");
					++testvar;
				}
			}

			COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0);
			COMPLIANCE_TEST(ERROR_OK == riscv_013_register_read_direct(target, &value, GDB_REGNO_S0),
				"Need to be able to read S0 to test ABSTRACTAUTO");

			COMPLIANCE_TEST(testvar == value,
				"ABSTRACTAUTO should cause COMMAND to run the expected number of times.");
		}
	}

	/* Single-Step each hart. */
	for (int hartsel = 0; hartsel < riscv_count_harts(target); ++hartsel) {
		COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, hartsel));
		COMPLIANCE_MUST_PASS(riscv_013_on_step(target));
		COMPLIANCE_MUST_PASS(riscv_013_step_current_hart(target));
		COMPLIANCE_TEST(riscv_halt_reason(target, hartsel) == RISCV_HALT_SINGLESTEP,
			"Single Step should result in SINGLESTEP");
	}

	/* Core Register Tests */
	uint64_t bogus_dpc = 0xdeadbeef;
	for (int hartsel = 0; hartsel < riscv_count_harts(target); ++hartsel) {
		COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, hartsel));

		/* DCSR Tests */
		COMPLIANCE_MUST_PASS(riscv_013_register_write_direct(target, GDB_REGNO_DCSR, 0x0));
		COMPLIANCE_MUST_PASS(riscv_013_register_read_direct(target, &value, GDB_REGNO_DCSR));
		COMPLIANCE_TEST(value != 0, "Not all bits in DCSR are writable by Debugger");
		COMPLIANCE_MUST_PASS(riscv_013_register_write_direct(target, GDB_REGNO_DCSR, 0xFFFFFFFF));
		COMPLIANCE_MUST_PASS(riscv_013_register_read_direct(target, &value, GDB_REGNO_DCSR));
		COMPLIANCE_TEST(value != 0, "At least some bits in DCSR must be 1");

		/* DPC. Note that DPC is sign-extended. */
		riscv_reg_t dpcmask = 0xFFFFFFFCUL;
		riscv_reg_t dpc;

		if (riscv_xlen(target) > 32)
			dpcmask |= (0xFFFFFFFFULL << 32);

		if (riscv_supports_extension(target, riscv_current_hartid(target), 'C'))
			dpcmask |= 0x2;

		COMPLIANCE_MUST_PASS(riscv_013_register_write_direct(target, GDB_REGNO_DPC, dpcmask));
		COMPLIANCE_MUST_PASS(riscv_013_register_read_direct(target, &dpc, GDB_REGNO_DPC));
		COMPLIANCE_TEST(dpcmask == dpc,
			"DPC must be sign-extended to XLEN and writable to all-1s (except the least significant bits)");
		COMPLIANCE_MUST_PASS(riscv_013_register_write_direct(target, GDB_REGNO_DPC, 0));
		COMPLIANCE_MUST_PASS(riscv_013_register_read_direct(target, &dpc, GDB_REGNO_DPC));
		COMPLIANCE_TEST(dpc == 0, "DPC must be writable to 0.");
		if (hartsel == 0)
			bogus_dpc = dpc; /* For a later test step */
	}

	/* NDMRESET
	Asserting non-debug module reset should not reset Debug Module state.
	But it should reset Hart State, e.g. DPC should get a different value.
	Also make sure that DCSR reports cause of 'HALT' even though previously we single-stepped.
	*/

	/* Write some registers. They should not be impacted by ndmreset. */
	COMPLIANCE_WRITE(target, DMI_COMMAND, 0xFFFFFFFF);

	for (unsigned i = 0; i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE); ++i) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_WRITE(target, DMI_PROGBUF0 + i, testvar);
	}

	for (unsigned i = 0; i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_DATACOUNT); ++i) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_WRITE(target, DMI_DATA0 + i, testvar);
	}

	COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0xFFFFFFFF);
	COMPLIANCE_READ(target, &abstractauto, DMI_ABSTRACTAUTO);

	/* Pulse reset. */
	target->reset_halt = true;
	COMPLIANCE_MUST_PASS(riscv_set_current_hartid(target, 0));
	COMPLIANCE_TEST(ERROR_OK == riscv_013_assert_reset(target), "Must be able to assert NDMRESET");
	COMPLIANCE_TEST(ERROR_OK == riscv_013_deassert_reset(target), "Must be able to deassert NDMRESET");

	/* Verify that most stuff is not affected by ndmreset. */
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
	COMPLIANCE_TEST(FIELD_GET(testvar_read, DMI_ABSTRACTCS_CMDERR) == CMDERR_NOT_SUPPORTED,
		"NDMRESET should not affect DMI_ABSTRACTCS");
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTAUTO);
	COMPLIANCE_TEST(testvar_read == abstractauto, "NDMRESET should not affect DMI_ABSTRACTAUTO");

	/* Clean up to avoid future test failures */
	COMPLIANCE_WRITE(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);
	COMPLIANCE_WRITE(target, DMI_ABSTRACTAUTO, 0);

	for (unsigned i = 0; i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE); ++i) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_READ(target, &testvar_read, DMI_PROGBUF0 + i);
		COMPLIANCE_TEST(testvar_read == testvar, "PROGBUF words must not be affected by NDMRESET");
	}

	for (unsigned i = 0; i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_DATACOUNT); ++i) {
		testvar = (i + 1) * 0x11111111;
		COMPLIANCE_READ(target, &testvar_read, DMI_DATA0 + i);
		COMPLIANCE_TEST(testvar_read == testvar, "DATA words must not be affected by NDMRESET");
	}

	/* Verify that DPC *is* affected by ndmreset. Since we don't know what it *should* be,
	just verify that at least it's not the bogus value anymore. */

	COMPLIANCE_TEST(bogus_dpc != 0xdeadbeef, "BOGUS DPC should have been set somehow (bug in compliance test)");
	COMPLIANCE_MUST_PASS(riscv_013_register_read_direct(target, &value, GDB_REGNO_DPC));
	COMPLIANCE_TEST(bogus_dpc != value, "NDMRESET should move DPC to reset value.");

	COMPLIANCE_TEST(riscv_halt_reason(target, 0) == RISCV_HALT_INTERRUPT,
		"After NDMRESET halt, DCSR should report cause of halt");

	/* DMACTIVE -- deasserting DMACTIVE should reset all the above values. */

	/* Toggle dmactive */
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, 0);
	COMPLIANCE_WRITE(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE);
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTCS);
	COMPLIANCE_TEST(FIELD_GET(testvar_read, DMI_ABSTRACTCS_CMDERR) == 0, "ABSTRACTCS.cmderr should reset to 0");
	COMPLIANCE_READ(target, &testvar_read, DMI_ABSTRACTAUTO);
	COMPLIANCE_TEST(testvar_read == 0, "ABSTRACTAUTO should reset to 0");

	for (unsigned i = 0; i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE); ++i) {
		COMPLIANCE_READ(target, &testvar_read, DMI_PROGBUF0 + i);
		COMPLIANCE_TEST(testvar_read == 0, "PROGBUF words should reset to 0");
	}

	for (unsigned i = 0; i < FIELD_GET(abstractcs, DMI_ABSTRACTCS_DATACOUNT); ++i) {
		COMPLIANCE_READ(target, &testvar_read, DMI_DATA0 + i);
		COMPLIANCE_TEST(testvar_read == 0, "DATA words should reset to 0");
	}

	/**
	@todo DCSR.cause priorities
	@todo DCSR.stoptime/stopcycle
	@todo DCSR.stepie
	@todo DCSR.ebreak
	@todo DCSR.prv
	*/

	/* Halt every hart for any follow-up tests*/
	COMPLIANCE_MUST_PASS(riscv_halt_all_harts(target));

	uint32_t failed_tests = total_tests - passed_tests;
	if (total_tests == passed_tests) {
		LOG_INFO("%s: ALL TESTS PASSED", target_name(target));
		return ERROR_OK;
	} else {
		LOG_INFO("%s: %d TESTS FAILED", target_name(target), failed_tests);
		return ERROR_TARGET_FAILURE;
	}
}

static int
riscv_013_arch_state(struct target *const target)
{
	return ERROR_OK;
}

/** @return signed value for ordering */
static int
cmp_csr_info(void const *p1, void const *p2)
{
	struct csr_info const *const pp1 = p1;
	struct csr_info const *const pp2 = p2;
	return
		(int)(pp1->number) -
		(int)(pp2->number);
}

/** @return error code*/
static int
riscv_init_registers(struct target *const target)
{
	assert(target);
	if (target->reg_cache) {
		if (target->reg_cache->reg_list)
			free(target->reg_cache->reg_list);

		free(target->reg_cache);
	}

	target->reg_cache = calloc(1, sizeof *target->reg_cache);
	assert(target->reg_cache);
	target->reg_cache->name = "RISC-V Registers";
	target->reg_cache->num_regs = GDB_REGNO_COUNT;

	if (expose_custom) {
		for (unsigned i = 0; expose_custom[i].low <= expose_custom[i].high; ++i) {
			for (
				unsigned number = expose_custom[i].low;
				number <= expose_custom[i].high;
				++number
				)
				++target->reg_cache->num_regs;
		}
	}

	LOG_DEBUG("%s: create register cache for %d registers",
		target_name(target),
		target->reg_cache->num_regs);

	target->reg_cache->reg_list =
		calloc(target->reg_cache->num_regs, sizeof(struct reg));
	assert(target->reg_cache->reg_list);

	static unsigned const max_reg_name_len = 12;
	riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);

	if (rvi->reg_names)
		free(rvi->reg_names);

	rvi->reg_names = calloc(target->reg_cache->num_regs, max_reg_name_len);
	assert(rvi->reg_names);

	typedef struct reg_feature reg_feature_t;
	static reg_feature_t const feature_cpu = {
		.name = "org.gnu.gdb.riscv.cpu"
	};

	static reg_feature_t const feature_fpu = {
		.name = "org.gnu.gdb.riscv.fpu"
	};

	static reg_feature_t const feature_csr = {
		.name = "org.gnu.gdb.riscv.csr"
	};

	static reg_feature_t const feature_virtual = {
		.name = "org.gnu.gdb.riscv.virtual"
	};

	static reg_feature_t const feature_custom = {
		.name = "org.gnu.gdb.riscv.custom"
	};

	typedef struct reg_data_type reg_data_type_t;
	static reg_data_type_t const type_ieee_single = {
		.type = REG_TYPE_IEEE_SINGLE,
		.id = "ieee_single"
	};

	static reg_data_type_t const type_ieee_double = {
		.type = REG_TYPE_IEEE_DOUBLE,
		.id = "ieee_double"
	};

	/* encoding.h does not contain the registers in sorted order. */
	qsort(csr_info, DIM(csr_info), sizeof *csr_info, cmp_csr_info);

	unsigned custom_range_index = 0;
	int custom_within_range = 0;

	riscv_reg_info_t *const shared_reg_info = calloc(1, sizeof(riscv_reg_info_t));
	assert(shared_reg_info);
	shared_reg_info->target = target;

	char *reg_name = rvi->reg_names;

	/*
	When gdb requests register N, gdb_get_register_packet() assumes
	that this is register at index N in reg_list.
	So if there are certain registers that don't exist,
	we need to leave holes in the list
	(or renumber, but it would be nice not to have yet another
	set of numbers to translate between).
	*/
	for (uint32_t number = 0; number < target->reg_cache->num_regs; ++number) {
		assert(target && target->reg_cache && target->reg_cache->reg_list && number < target->reg_cache->num_regs);
		struct reg *const p_reg = &target->reg_cache->reg_list[number];
		p_reg->dirty = false;
		p_reg->valid = false;
		p_reg->exist = true;
		p_reg->type = &riscv_reg_arch_type;
		p_reg->arch_info = shared_reg_info;
		p_reg->number = number;
		p_reg->size = riscv_xlen(target);

		/* p_reg->size is set in riscv_invalidate_register_cache, maybe because the
		* target is in theory allowed to change XLEN on us. But I expect a lot
		* of other things to break in that case as well. */
		if (number <= GDB_REGNO_XPR31) {
			p_reg->caller_save = true;

			switch (number) {
			case GDB_REGNO_ZERO:
				p_reg->name = "zero";
				break;

			case GDB_REGNO_RA:
				p_reg->name = "ra";
				break;

			case GDB_REGNO_SP:
				p_reg->name = "sp";
				break;

			case GDB_REGNO_GP:
				p_reg->name = "gp";
				break;

			case GDB_REGNO_TP:
				p_reg->name = "tp";
				break;

			case GDB_REGNO_T0:
				p_reg->name = "t0";
				break;

			case GDB_REGNO_T1:
				p_reg->name = "t1";
				break;

			case GDB_REGNO_T2:
				p_reg->name = "t2";
				break;

			case GDB_REGNO_FP:
				p_reg->name = "fp";
				break;

			case GDB_REGNO_S1:
				p_reg->name = "s1";
				break;

			case GDB_REGNO_A0:
				p_reg->name = "a0";
				break;

			case GDB_REGNO_A1:
				p_reg->name = "a1";
				break;

			case GDB_REGNO_A2:
				p_reg->name = "a2";
				break;

			case GDB_REGNO_A3:
				p_reg->name = "a3";
				break;

			case GDB_REGNO_A4:
				p_reg->name = "a4";
				break;

			case GDB_REGNO_A5:
				p_reg->name = "a5";
				break;

			case GDB_REGNO_A6:
				p_reg->name = "a6";
				break;

			case GDB_REGNO_A7:
				p_reg->name = "a7";
				break;

			case GDB_REGNO_S2:
				p_reg->name = "s2";
				break;

			case GDB_REGNO_S3:
				p_reg->name = "s3";
				break;

			case GDB_REGNO_S4:
				p_reg->name = "s4";
				break;

			case GDB_REGNO_S5:
				p_reg->name = "s5";
				break;

			case GDB_REGNO_S6:
				p_reg->name = "s6";
				break;

			case GDB_REGNO_S7:
				p_reg->name = "s7";
				break;

			case GDB_REGNO_S8:
				p_reg->name = "s8";
				break;

			case GDB_REGNO_S9:
				p_reg->name = "s9";
				break;

			case GDB_REGNO_S10:
				p_reg->name = "s10";
				break;

			case GDB_REGNO_S11:
				p_reg->name = "s11";
				break;

			case GDB_REGNO_T3:
				p_reg->name = "t3";
				break;

			case GDB_REGNO_T4:
				p_reg->name = "t4";
				break;

			case GDB_REGNO_T5:
				p_reg->name = "t5";
				break;

			case GDB_REGNO_T6:
				p_reg->name = "t6";
				break;
			}

			p_reg->group = "general";
			/**
			@todo This should probably be const.
			*/
			p_reg->feature = (reg_feature_t *)(&feature_cpu);
		} else if (number == GDB_REGNO_PC) {
			p_reg->caller_save = true;
			reg_name[max_reg_name_len - 1] = '\0';
			snprintf(reg_name, max_reg_name_len - 1, "pc");
			p_reg->group = "general";
			/**
			@todo This should probably be const.
			*/
			p_reg->feature = (reg_feature_t *)(&feature_cpu);
		} else if (GDB_REGNO_FPR0 <= number && number <= GDB_REGNO_FPR31) {
			p_reg->caller_save = true;

			if (riscv_supports_extension(target, riscv_current_hartid(target), 'D')) {
				/**
				@todo This should probably be const.
				*/
				p_reg->reg_data_type = (reg_data_type_t *)(&type_ieee_double);
				p_reg->size = 64;
			} else if (riscv_supports_extension(target, riscv_current_hartid(target), 'F')) {
				/**
				@todo This should probably be const.
				*/
				p_reg->reg_data_type = (reg_data_type_t *)(&type_ieee_single);
				p_reg->size = 32;
			} else {
				p_reg->exist = false;
			}

			switch (number) {
			case GDB_REGNO_FT0:
				p_reg->name = "ft0";
				break;

			case GDB_REGNO_FT1:
				p_reg->name = "ft1";
				break;

			case GDB_REGNO_FT2:
				p_reg->name = "ft2";
				break;

			case GDB_REGNO_FT3:
				p_reg->name = "ft3";
				break;

			case GDB_REGNO_FT4:
				p_reg->name = "ft4";
				break;

			case GDB_REGNO_FT5:
				p_reg->name = "ft5";
				break;

			case GDB_REGNO_FT6:
				p_reg->name = "ft6";
				break;

			case GDB_REGNO_FT7:
				p_reg->name = "ft7";
				break;

			case GDB_REGNO_FS0:
				p_reg->name = "fs0";
				break;

			case GDB_REGNO_FS1:
				p_reg->name = "fs1";
				break;

			case GDB_REGNO_FA0:
				p_reg->name = "fa0";
				break;

			case GDB_REGNO_FA1:
				p_reg->name = "fa1";
				break;

			case GDB_REGNO_FA2:
				p_reg->name = "fa2";
				break;

			case GDB_REGNO_FA3:
				p_reg->name = "fa3";
				break;

			case GDB_REGNO_FA4:
				p_reg->name = "fa4";
				break;

			case GDB_REGNO_FA5:
				p_reg->name = "fa5";
				break;

			case GDB_REGNO_FA6:
				p_reg->name = "fa6";
				break;

			case GDB_REGNO_FA7:
				p_reg->name = "fa7";
				break;

			case GDB_REGNO_FS2:
				p_reg->name = "fs2";
				break;

			case GDB_REGNO_FS3:
				p_reg->name = "fs3";
				break;

			case GDB_REGNO_FS4:
				p_reg->name = "fs4";
				break;

			case GDB_REGNO_FS5:
				p_reg->name = "fs5";
				break;

			case GDB_REGNO_FS6:
				p_reg->name = "fs6";
				break;

			case GDB_REGNO_FS7:
				p_reg->name = "fs7";
				break;

			case GDB_REGNO_FS8:
				p_reg->name = "fs8";
				break;

			case GDB_REGNO_FS9:
				p_reg->name = "fs9";
				break;

			case GDB_REGNO_FS10:
				p_reg->name = "fs10";
				break;

			case GDB_REGNO_FS11:
				p_reg->name = "fs11";
				break;

			case GDB_REGNO_FT8:
				p_reg->name = "ft8";
				break;

			case GDB_REGNO_FT9:
				p_reg->name = "ft9";
				break;

			case GDB_REGNO_FT10:
				p_reg->name = "ft10";
				break;

			case GDB_REGNO_FT11:
				p_reg->name = "ft11";
				break;
				/**
				@bug no default case
				*/
			}

			p_reg->group = "float";
			/**
			@todo This should probably be const.
			*/
			p_reg->feature = (reg_feature_t *)(&feature_fpu);
		} else if (GDB_REGNO_CSR0 <= number && number <= GDB_REGNO_CSR4095) {
			p_reg->group = "csr";
			/**
			@todo This should probably be const.
			*/
			p_reg->feature = (reg_feature_t *)(&feature_csr);
			unsigned const csr_number = number - GDB_REGNO_CSR0;

			unsigned csr_info_index = 0;
			while (csr_info[csr_info_index].number < csr_number && csr_info_index < DIM(csr_info) - 1)
				++csr_info_index;

			if (csr_info[csr_info_index].number == csr_number) {
				p_reg->name = csr_info[csr_info_index].name;
			} else {
				reg_name[max_reg_name_len - 1] = '\0';
				snprintf(reg_name, max_reg_name_len - 1, "csr%d", csr_number);
				/* Assume unnamed registers don't exist, unless we have some
				* configuration that tells us otherwise. That's important
				* because eg. Eclipse crashes if a target has too many
				* registers, and apparently has no way of only showing a
				* subset of registers in any case. */
				p_reg->exist = false;
			}

			switch (csr_number) {
			case CSR_FFLAGS:
			case CSR_FRM:
			case CSR_FCSR:
				p_reg->exist =
					riscv_supports_extension(target, riscv_current_hartid(target), 'F');
				p_reg->group = "float";
				/**
				@todo This should probably be const.
				*/
				p_reg->feature = (reg_feature_t *)&feature_fpu;
				break;

			case CSR_SSTATUS:
			case CSR_STVEC:
			case CSR_SIP:
			case CSR_SIE:
			case CSR_SCOUNTEREN:
			case CSR_SSCRATCH:
			case CSR_SEPC:
			case CSR_SCAUSE:
			case CSR_STVAL:
			case CSR_SATP:
				p_reg->exist = riscv_supports_extension(target,
					riscv_current_hartid(target), 'S');
				break;

			case CSR_MEDELEG:
			case CSR_MIDELEG:
				/* "In systems with only M-mode, or with both M-mode and
				* U-mode but without U-mode trap support, the medeleg and
				* mideleg registers should not exist." */
				p_reg->exist = riscv_supports_extension(target, riscv_current_hartid(target), 'S') ||
					riscv_supports_extension(target, riscv_current_hartid(target), 'N');
				break;

			case CSR_CYCLEH:
			case CSR_TIMEH:
			case CSR_INSTRETH:
			case CSR_HPMCOUNTER3H:
			case CSR_HPMCOUNTER4H:
			case CSR_HPMCOUNTER5H:
			case CSR_HPMCOUNTER6H:
			case CSR_HPMCOUNTER7H:
			case CSR_HPMCOUNTER8H:
			case CSR_HPMCOUNTER9H:
			case CSR_HPMCOUNTER10H:
			case CSR_HPMCOUNTER11H:
			case CSR_HPMCOUNTER12H:
			case CSR_HPMCOUNTER13H:
			case CSR_HPMCOUNTER14H:
			case CSR_HPMCOUNTER15H:
			case CSR_HPMCOUNTER16H:
			case CSR_HPMCOUNTER17H:
			case CSR_HPMCOUNTER18H:
			case CSR_HPMCOUNTER19H:
			case CSR_HPMCOUNTER20H:
			case CSR_HPMCOUNTER21H:
			case CSR_HPMCOUNTER22H:
			case CSR_HPMCOUNTER23H:
			case CSR_HPMCOUNTER24H:
			case CSR_HPMCOUNTER25H:
			case CSR_HPMCOUNTER26H:
			case CSR_HPMCOUNTER27H:
			case CSR_HPMCOUNTER28H:
			case CSR_HPMCOUNTER29H:
			case CSR_HPMCOUNTER30H:
			case CSR_HPMCOUNTER31H:
			case CSR_MCYCLEH:
			case CSR_MINSTRETH:
			case CSR_MHPMCOUNTER3H:
			case CSR_MHPMCOUNTER4H:
			case CSR_MHPMCOUNTER5H:
			case CSR_MHPMCOUNTER6H:
			case CSR_MHPMCOUNTER7H:
			case CSR_MHPMCOUNTER8H:
			case CSR_MHPMCOUNTER9H:
			case CSR_MHPMCOUNTER10H:
			case CSR_MHPMCOUNTER11H:
			case CSR_MHPMCOUNTER12H:
			case CSR_MHPMCOUNTER13H:
			case CSR_MHPMCOUNTER14H:
			case CSR_MHPMCOUNTER15H:
			case CSR_MHPMCOUNTER16H:
			case CSR_MHPMCOUNTER17H:
			case CSR_MHPMCOUNTER18H:
			case CSR_MHPMCOUNTER19H:
			case CSR_MHPMCOUNTER20H:
			case CSR_MHPMCOUNTER21H:
			case CSR_MHPMCOUNTER22H:
			case CSR_MHPMCOUNTER23H:
			case CSR_MHPMCOUNTER24H:
			case CSR_MHPMCOUNTER25H:
			case CSR_MHPMCOUNTER26H:
			case CSR_MHPMCOUNTER27H:
			case CSR_MHPMCOUNTER28H:
			case CSR_MHPMCOUNTER29H:
			case CSR_MHPMCOUNTER30H:
			case CSR_MHPMCOUNTER31H:
				p_reg->exist = riscv_xlen(target) == 32;
				break;
			}

			if (!p_reg->exist && expose_csr) {
				for (unsigned i = 0; expose_csr[i].low <= expose_csr[i].high; ++i) {
					if (csr_number >= expose_csr[i].low && csr_number <= expose_csr[i].high) {
						LOG_INFO("%s: Exposing additional CSR %d", target_name(target), csr_number);
						p_reg->exist = true;
						break;
					}
				}
			}

		} else if (number == GDB_REGNO_PRIV) {
			reg_name[max_reg_name_len - 1] = '\0';
			snprintf(reg_name, max_reg_name_len - 1, "priv");
			p_reg->group = "general";
			/**
			@todo This should probably be const.
			*/
			p_reg->feature = (reg_feature_t *)(&feature_virtual);
			p_reg->size = 8;

		} else {
			/* Custom registers. */
			assert(expose_custom);

			range_t *const range = &expose_custom[custom_range_index];
			assert(range->low <= range->high);
			unsigned const custom_number = range->low + custom_within_range;

			p_reg->group = "custom";
			/**
			@todo This should probably be const.
			*/
			p_reg->feature = (reg_feature_t *)(&feature_custom);
			p_reg->arch_info = calloc(1, sizeof(riscv_reg_info_t));
			assert(p_reg->arch_info);
			((riscv_reg_info_t *)(p_reg->arch_info))->target = target;
			((riscv_reg_info_t *)(p_reg->arch_info))->custom_number = custom_number;
			reg_name[max_reg_name_len - 1] = '\0';
			snprintf(reg_name, max_reg_name_len - 1, "custom%d", custom_number);

			++custom_within_range;

			if (custom_within_range > range->high - range->low) {
				custom_within_range = 0;
				++custom_range_index;
			}
		}

		if (*reg_name)
			p_reg->name = reg_name;

		reg_name += strlen(reg_name) + 1;
		assert(reg_name < rvi->reg_names + target->reg_cache->num_regs * max_reg_name_len);
		p_reg->value = &rvi->reg_cache_values[number];
	}

	return ERROR_OK;
}

static int
riscv_013_examine(struct target *const target)
{
	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */

	uint32_t dtmcontrol;
	{
		int const error_code = dtmcontrol_scan(target->tap, 0, &dtmcontrol);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: fatal: examine failure, JTAG/TAP error",
				target_name(target));
			return error_code;
		}
	}

	assert(target);
	LOG_DEBUG("%s: dtmcontrol=0x%" PRIx32 " ("
		"dmireset=%" PRId32
		" idle=%" PRId32
		" dmistat=%" PRId32
		" abits = %" PRId32
		" version=%" PRId32
		")",
		target_name(target),
		dtmcontrol,
		FIELD_GET(dtmcontrol, DTM_DTMCS_DMIRESET),
		FIELD_GET(dtmcontrol, DTM_DTMCS_IDLE),
		FIELD_GET(dtmcontrol, DTM_DTMCS_DMISTAT),
		FIELD_GET(dtmcontrol, DTM_DTMCS_ABITS),
		FIELD_GET(dtmcontrol, DTM_DTMCS_VERSION));

	if (dtmcontrol == 0) {
		LOG_ERROR("%s: dtmcontrol is 0. Check JTAG connectivity/board power.", target_name(target));
		return ERROR_TARGET_FAILURE;
	}

	if (1 != FIELD_GET(dtmcontrol, DTM_DTMCS_VERSION)) {
		LOG_ERROR("%s: Unsupported DTM version %" PRId32 " (dtmcontrol=0x%" PRIx32 ")",
			target_name(target), FIELD_GET(dtmcontrol, DTM_DTMCS_VERSION), dtmcontrol);
		return ERROR_TARGET_INVALID;
	}

	riscv_013_info_t *const info = get_info(target);
	assert(info);
	info->abits = FIELD_GET(dtmcontrol, DTM_DTMCS_ABITS);
	info->dtmcontrol_idle = FIELD_GET(dtmcontrol, DTM_DTMCS_IDLE);

	uint32_t dmstatus;
	{
		int const error_code = dmstatus_read(target, &dmstatus, false);

		if (ERROR_OK != error_code)
			return error_code;
	}

	LOG_DEBUG("%s: dmstatus:  0x%08x", target_name(target), dmstatus);

	if (FIELD_GET(dmstatus, DMI_DMSTATUS_VERSION) != 2) {
		LOG_ERROR("%s: OpenOCD only supports Debug Module version 2, not %" PRId32 " (dmstatus=0x%" PRIx32 ")",
			target_name(target), FIELD_GET(dmstatus, DMI_DMSTATUS_VERSION), dmstatus);
		return ERROR_TARGET_FAILURE;
	}

	/* Reset the Debug Module. */
	dm013_info_t *dm = get_dm(target);
	assert(dm);

	if (!dm->was_reset) {
		int error_code;

		if (!(
			ERROR_OK == (error_code = dmi_write(target, DMI_DMCONTROL, 0)) &&
			ERROR_OK == (error_code = dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE))
			))
			return error_code;

		dm->was_reset = true;
	}

	{
		int const error_code =
			dmi_write(target, DMI_DMCONTROL,
				DMI_DMCONTROL_HARTSELLO |
				DMI_DMCONTROL_HARTSELHI |
				DMI_DMCONTROL_DMACTIVE);

		if (ERROR_OK != error_code)
			return error_code;
	}

	uint32_t dmcontrol;
	{
		int const error_code = dmi_read(target, &dmcontrol, DMI_DMCONTROL);

		if (ERROR_OK != error_code)
			return error_code;
	}

	if (!FIELD_GET(dmcontrol, DMI_DMCONTROL_DMACTIVE)) {
		LOG_ERROR("%s: Debug Module did not become active. dmcontrol=0x%x",
			target_name(target), dmcontrol);
		return ERROR_TARGET_FAILURE;
	}

	uint32_t hartsel =
		FIELD_GET(dmcontrol, DMI_DMCONTROL_HARTSELHI) << DMI_DMCONTROL_HARTSELLO_LENGTH |
		FIELD_GET(dmcontrol, DMI_DMCONTROL_HARTSELLO);
	info->hartsellen = 0;

	while (hartsel & 1) {
		++info->hartsellen;
		hartsel >>= 1;
	}
	LOG_DEBUG("%s: hartsellen=%d", target_name(target), info->hartsellen);

	uint32_t hartinfo;

	{
		int const error_code = dmi_read(target, &hartinfo, DMI_HARTINFO);

		if (ERROR_OK != error_code)
			return error_code;
	}

	info->datasize = FIELD_GET(hartinfo, DMI_HARTINFO_DATASIZE);
	info->dataaccess = FIELD_GET(hartinfo, DMI_HARTINFO_DATAACCESS);
	info->dataaddr = FIELD_GET(hartinfo, DMI_HARTINFO_DATAADDR);

	if (!FIELD_GET(dmstatus, DMI_DMSTATUS_AUTHENTICATED)) {
		LOG_ERROR("%s: Debugger is not authenticated to target Debug Module (dmstatus=0x%x)."
			" Use `riscv authdata_read` and `riscv authdata_write` commands to authenticate.",
			target_name(target), dmstatus);
		/**
		@todo If we return ERROR_FAIL here,
		then in a multicore setup the next core won't be examined,
		which means we won't set up the authentication commands for them,
		which means the config script needs to be a lot more complex.
		*/
		return ERROR_OK;
	}

	{
		int const error_code = dmi_read(target, &info->sbcs, DMI_SBCS);

		if (ERROR_OK != error_code)
			return error_code;
	}

	/* Check that abstract data registers are accessible. */
	uint32_t abstractcs;

	{
		int const error_code = dmi_read(target, &abstractcs, DMI_ABSTRACTCS);

		if (ERROR_OK != error_code)
			return error_code;
	}

	info->datacount = FIELD_GET(abstractcs, DMI_ABSTRACTCS_DATACOUNT);
	info->progbufsize = FIELD_GET(abstractcs, DMI_ABSTRACTCS_PROGBUFSIZE);

	LOG_INFO("%s: datacount=%d progbufsize=%d", target_name(target), info->datacount, info->progbufsize);

	riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);
	rvi->impebreak = FIELD_GET(dmstatus, DMI_DMSTATUS_IMPEBREAK);

	if (info->progbufsize + rvi->impebreak < 2) {
		LOG_WARNING("%s: We won't be able to execute fence instructions on this "
			"target. Memory may not always appear consistent. "
			"(progbufsize=%d, impebreak=%d)",
			target_name(target), info->progbufsize, rvi->impebreak);
	}

	/* Before doing anything else we must first enumerate the harts. */

	/* Don't call any riscv_* functions until after we've counted the number of
	* cores and initialized registers. */
	for (int i = 0; i < MIN(RISCV_MAX_HARTS, 1 << info->hartsellen); ++i) {
		if (!riscv_rtos_enabled(target) && i != target->coreid)
			continue;

		rvi->current_hartid = i;

		{
			int const error_code = riscv_013_select_current_hart(target);

			if (ERROR_OK != error_code)
				return error_code;
		}

		uint32_t s;

		{
			int const error_code = dmstatus_read(target, &s, true);

			if (ERROR_OK != error_code)
				return error_code;
		}

		if (FIELD_GET(s, DMI_DMSTATUS_ANYNONEXISTENT))
			break;

		rvi->hart_count = i + 1;

		if (FIELD_GET(s, DMI_DMSTATUS_ANYHAVERESET)) {
			int const error_code =
				dmi_write(target, DMI_DMCONTROL, set_hartsel(DMI_DMCONTROL_DMACTIVE | DMI_DMCONTROL_ACKHAVERESET, i));

			if (ERROR_OK != error_code)
				return error_code;
		}

		bool const halted = riscv_is_halted(target);
		if (!halted) {
			int const error_code = riscv_013_halt_current_hart(target);

			if (ERROR_OK != error_code) {
				LOG_ERROR("%s: Fatal: Hart %d failed to halt during examine()", target_name(target), i);
				return error_code;
			}
		}

		/* Without knowing anything else we can at least mess with the
		* program buffer. */
		rvi->harts[i].debug_buffer_size = info->progbufsize;

		{
			int const result = register_read_abstract(target, NULL, GDB_REGNO_S0, 64);
			/**
			@todo Support 128
			*/
			rvi->harts[i].xlen = ERROR_OK == result ? 64 : 32;
		}

		{
			int const error_code = riscv_013_register_read(target, &rvi->harts[i].misa, GDB_REGNO_MISA);

			if (ERROR_OK != error_code) {
				LOG_ERROR("%s: Fatal: Failed to read MISA from hart %d.",
					target_name(target), i);
				return error_code;
			}
		}

		{
			/* Now init registers based on what we discovered. */
			int const error_code = riscv_init_registers(target);

			if (ERROR_OK != error_code)
				return error_code;
		}

		/* Display this as early as possible to help people who are using
		* really slow simulators. */
		LOG_DEBUG("%s: hart %d: XLEN=%d, misa=0x%" PRIx64,
			target_name(target), i, rvi->harts[i].xlen, rvi->harts[i].misa);

		if (!halted)
			riscv_013_resume_current_hart(target);
	}

	LOG_DEBUG("%s: Enumerated %d harts",
		target_name(target), rvi->hart_count);

	if (0 == rvi->hart_count) {
		LOG_ERROR("%s: No harts found!", target_name(target));
		return ERROR_TARGET_INVALID;
	}

	target_set_examined(target);

	if (target->rtos)
		riscv_update_threads(target->rtos);

	/* Some regression suites rely on seeing 'Examined RISC-V core' to know
	* when they can connect with gdb/telnet.
	* We will need to update those suites if we want to change that text. */
	LOG_INFO("%s: Examined RISC-V core; found %d harts", target_name(target),
		riscv_count_harts(target));

	for (int i = 0; i < riscv_count_harts(target); ++i) {
		if (riscv_hart_enabled(target, i)) {
			LOG_INFO("%s: hart %d: XLEN=%d, misa=0x%" PRIx64,
				target_name(target), i, rvi->harts[i].xlen, rvi->harts[i].misa);
		} else {
			LOG_INFO("%s: hart %d: currently disabled",
				target_name(target), i);
		}
	}

	return ERROR_OK;
}

static int
riscv_013_authdata_write(struct target *const target,
	uint32_t value)
{
	uint32_t before;

	{
		int const error_code = wait_for_authbusy(target, &before);

		if (ERROR_OK != error_code)
			return error_code;
	}

	{
		int const error_code = dmi_write(target, DMI_AUTHDATA, value);

		if (ERROR_OK != error_code)
			return error_code;
	}

	uint32_t after;

	{
		int const error_code = wait_for_authbusy(target, &after);

		if (ERROR_OK != error_code)
			return error_code;
	}

	if (0 == FIELD_GET(before, DMI_DMSTATUS_AUTHENTICATED) &&
		0 != FIELD_GET(after, DMI_DMSTATUS_AUTHENTICATED)) {
		LOG_INFO("%s: authdata_write resulted in successful authentication", target_name(target));
		int result = ERROR_OK;
		dm013_info_t *dm = get_dm(target);
		target_list_t *entry;

		list_for_each_entry(entry, &dm->target_list, list)
		{
			int const err = riscv_013_examine(entry->target);
			result = ERROR_OK == result ? err : result;
		}

		return result;
	}

	return ERROR_OK;
}

static int
riscv_013_init_target(struct command_context *const cmd_ctx,
	struct target *const target)
{
	assert(target);
	LOG_DEBUG("%s: init", target_name(target));
	riscv_info_t *const generic_info = target->arch_info;
	assert(generic_info);

	generic_info->version_specific = calloc(1, sizeof(riscv_013_info_t));

	if (!generic_info->version_specific) {
		LOG_ERROR("%s: Can't allocate memory", target_name(target));
		return ERROR_TARGET_INIT_FAILED;
	}

	riscv_013_info_t *const info = get_info(target);
	assert(info);

	info->progbufsize = -1;
	info->dmi_busy_delay = 0;
	info->bus_master_read_delay = 0;
	info->bus_master_write_delay = 0;
	info->ac_busy_delay = 0;

	/* Assume all these abstract commands are supported until we learn
	* otherwise. */
	/**
	@todo The spec allows eg. one CSR to be able to be accessed abstractly
	while another one isn't.
	We don't track that this closely here, but in the future we probably should.
	*/
	info->abstract_read_csr_supported = true;
	info->abstract_write_csr_supported = true;
	info->abstract_read_fpr_supported = true;
	info->abstract_write_fpr_supported = true;

	return ERROR_OK;
}

/** Create the shared RISC-V structure. 
	@see riscv_info_t
*/
static riscv_info_t *
__attribute__((warn_unused_result))
riscv_info_init(struct target *const target)
{
	riscv_info_t *const r = calloc(1, sizeof(riscv_info_t));

	if (!r) {
		LOG_ERROR("%s: Fatal: No free memory!", target_name(target));
		return NULL;
	}

	r->dtm_version = 1;
	r->registers_initialized = false;
	/**
	@bug r->current_hartid != target->coreid in common case
	*/
	r->current_hartid = target->coreid;

	memset(r->trigger_unique_id, 0xff, sizeof r->trigger_unique_id);

	for (size_t hart = 0; hart < RISCV_MAX_HARTS; ++hart) {
		r->harts[hart].xlen = -1;
	}

	return r;
}

/** @return error code */
static int
riscv_init_target(struct command_context *const cmd_ctx,
	struct target *const target)
{
	assert(target);
	LOG_DEBUG("%s: riscv_init_target()", target_name(target));
	target->arch_info = riscv_info_init(target);

	if (!target->arch_info) {
		LOG_ERROR("%s: Can't init arch_info", target_name(target));
		return ERROR_TARGET_INVALID;
	}

	riscv_info_t *const info = target->arch_info;
	info->cmd_ctx = cmd_ctx;

	assert(target->tap);
	riscv_semihosting_init(target);

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static void
riscv_deinit_target(struct target *const target)
{
	LOG_DEBUG("%s: riscv_deinit_target()", target_name(target));
	riscv_013_deinit_target(target);
	riscv_info_t *const info = target->arch_info;
	free(info->reg_names);
	free(info);

	/* Free the shared structure use for most registers. */
	assert(target->reg_cache->reg_list && 0 < target->reg_cache->num_regs);
	free(target->reg_cache->reg_list[0].arch_info);

	/* Free the ones we allocated separately. */
	for (unsigned i = GDB_REGNO_COUNT; i < target->reg_cache->num_regs; ++i)
		free(target->reg_cache->reg_list[i].arch_info);

	free(target->reg_cache->reg_list);
	free(target->reg_cache);
	target->arch_info = NULL;
}

static void
trigger_from_breakpoint(struct trigger *const trigger,
		struct breakpoint const *const breakpoint)
{
	assert(trigger && breakpoint);
	trigger->address = breakpoint->address;
	trigger->length = breakpoint->length;
	trigger->mask = ~UINT64_C(0);
	trigger->read = false;
	trigger->write = false;
	trigger->execute = true;
	/* unique_id is unique across both breakpoints and watchpoints. */
	trigger->unique_id = breakpoint->unique_id;
}

/** @return error code */
static int
maybe_add_trigger_t1(struct target *const target,
	unsigned const hartid,
	struct trigger *const trigger,
	uint64_t tdata1)
{
	static uint32_t const bpcontrol_x = 1 << 0;
	static uint32_t const bpcontrol_w = 1 << 1;
	static uint32_t const bpcontrol_r = 1 << 2;
	static uint32_t const bpcontrol_u = 1 << 3;
	static uint32_t const bpcontrol_s = 1 << 4;
	static uint32_t const bpcontrol_h = 1 << 5;
	static uint32_t const bpcontrol_m = 1 << 6;
	static uint32_t const bpcontrol_bpmatch = 0xf << 7;
	static uint32_t const bpcontrol_bpaction = 0xff << 11;

	if (tdata1 & (bpcontrol_r | bpcontrol_w | bpcontrol_x)) {
		/* Trigger is already in use, presumably by user code. */
		LOG_ERROR("%s: Trigger is already in use, presumably by user code", target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(trigger);
	riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);
	tdata1 = FIELD_SET(tdata1, bpcontrol_r, trigger->read);
	tdata1 = FIELD_SET(tdata1, bpcontrol_w, trigger->write);
	tdata1 = FIELD_SET(tdata1, bpcontrol_x, trigger->execute);
	tdata1 = FIELD_SET(tdata1, bpcontrol_u, !!(rvi->harts[hartid].misa & (1 << ('U' - 'A'))));
	tdata1 = FIELD_SET(tdata1, bpcontrol_s, !!(rvi->harts[hartid].misa & (1 << ('S' - 'A'))));
	tdata1 = FIELD_SET(tdata1, bpcontrol_h, !!(rvi->harts[hartid].misa & (1 << ('H' - 'A'))));
	tdata1 |= bpcontrol_m;
	tdata1 = FIELD_SET(tdata1, bpcontrol_bpmatch, 0); /* exact match */
	tdata1 = FIELD_SET(tdata1, bpcontrol_bpaction, 0); /* cause bp exception */

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);

	riscv_reg_t tdata1_rb;
	{
		int const error_code = riscv_get_register_on_hart(target, &tdata1_rb, hartid, GDB_REGNO_TDATA1);

		if (ERROR_OK != error_code)
			return error_code;
	}
	LOG_DEBUG("%s: tdata1=0x%" PRIx64, target_name(target), tdata1_rb);

	if (tdata1 != tdata1_rb) {
		LOG_DEBUG("%s: Trigger doesn't support what we need."
			"After writing 0x%" PRIx64 " to tdata1 it contains 0x%" PRIx64,
				target_name(target), tdata1, tdata1_rb);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA2, trigger->address);

	return ERROR_OK;
}

/** @return error code */
static int
maybe_add_trigger_t2(struct target *const target,
	unsigned const hartid,
	struct trigger *const trigger,
	uint64_t tdata1)
{
	riscv_info_t const *const rvi = riscv_info(target);

	/* tselect is already set */
	if (0 != (tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD))) {
		/* Trigger is already in use, presumably by user code. */
		LOG_ERROR("%s: Trigger is already in use, presumably by user code", target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* address/data match trigger */
	tdata1 |= MCONTROL_DMODE(riscv_xlen(target));
	tdata1 = FIELD_SET(tdata1, MCONTROL_ACTION, MCONTROL_ACTION_DEBUG_MODE);
	tdata1 = FIELD_SET(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_EQUAL);
	tdata1 |= MCONTROL_M;

	if (rvi->harts[hartid].misa & (1 << ('H' - 'A')))
		tdata1 |= MCONTROL_H;

	if (rvi->harts[hartid].misa & (1 << ('S' - 'A')))
		tdata1 |= MCONTROL_S;

	if (rvi->harts[hartid].misa & (1 << ('U' - 'A')))
		tdata1 |= MCONTROL_U;

	assert(trigger);
	if (trigger->execute)
		tdata1 |= MCONTROL_EXECUTE;

	if (trigger->read)
		tdata1 |= MCONTROL_LOAD;

	if (trigger->write)
		tdata1 |= MCONTROL_STORE;

	{
		int const error_code = riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);
		if (ERROR_OK != error_code)
			return error_code;
	}

	uint64_t tdata1_rb;
	{
		int const error_code =
			riscv_get_register_on_hart(target, &tdata1_rb, hartid, GDB_REGNO_TDATA1);
		if (ERROR_OK != error_code)
			return error_code;
	}

	LOG_DEBUG("%s: tdata1=0x%" PRIx64, target_name(target), tdata1_rb);

	if (tdata1 != tdata1_rb) {
		LOG_DEBUG("%s: Trigger doesn't support what we need; After writing 0x%"
				PRIx64 " to tdata1 it contains 0x%" PRIx64,
				target_name(target),
				tdata1, tdata1_rb);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA2, trigger->address);
}

/** @return error code */
static int
add_trigger(struct target *const target,
	struct trigger *const trigger)
{
	{
		int const error_code = riscv_enumerate_triggers(target);
		if (ERROR_OK != error_code)
			return error_code;
	}

	/** @details In RTOS mode, we need to set the same trigger in the same slot on every hart,
	to keep up the illusion that each hart is a thread running on the same core. 
	
	Otherwise, we just set the trigger on the one hart this target deals with.
	*/

	riscv_reg_t tselect[RISCV_MAX_HARTS];

	int first_hart = -1;
	int const number_of_harts = riscv_count_harts(target);

	for (int hartid = 0; hartid < number_of_harts; ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		if (first_hart < 0)
			first_hart = hartid;

		{
			int const error_code =
				riscv_get_register_on_hart(target, &tselect[hartid], hartid, GDB_REGNO_TSELECT);

			if (ERROR_OK != error_code)
				return error_code;
		}
	}

	assert(first_hart >= 0);

	riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);

	unsigned i;

	for (i = 0; i < rvi->harts[first_hart].trigger_count; ++i) {
		if (rvi->trigger_unique_id[i] != -1)
			continue;

		riscv_set_register_on_hart(target, first_hart, GDB_REGNO_TSELECT, i);

		uint64_t tdata1;
		{
			int const error_code =
				riscv_get_register_on_hart(target, &tdata1, first_hart, GDB_REGNO_TDATA1);

			if (ERROR_OK != error_code)
				return error_code;
		}

		int const type =
			FIELD_GET(tdata1, MCONTROL_TYPE(riscv_xlen(target)));

		{
			int error_code = ERROR_OK;

			for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
				if (!riscv_hart_enabled(target, hartid))
					continue;

				if (hartid > first_hart)
					riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, i);

				switch (type) {
				case 1:
					error_code = maybe_add_trigger_t1(target, hartid, trigger, tdata1);
					break;

				case 2:
					error_code = maybe_add_trigger_t2(target, hartid, trigger, tdata1);
					break;

				default:
					LOG_DEBUG("%s: trigger %d has unknown type %d", target_name(target), i, type);
					continue;
				}

				if (ERROR_OK != error_code)
					continue;
			}

			if (ERROR_OK != error_code)
				continue;
		}

		assert(trigger);
		LOG_DEBUG("%s: Using trigger %d (type %d) for bp %d",
				target_name(target),
				i,
				type,
				trigger->unique_id);
		rvi->trigger_unique_id[i] = trigger->unique_id;
		break;
	}

	for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		int const error_code =
			riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect[hartid]);

		if (ERROR_OK != error_code)
			return error_code;
	}

	if (rvi->harts[first_hart].trigger_count <= i) {
		LOG_ERROR("%s: Couldn't find an available hardware trigger.", target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return ERROR_OK;
}

/** @return error code */
static int
riscv_add_breakpoint(struct target *const target,
	struct breakpoint *const breakpoint)
{
	assert(breakpoint);

	switch (breakpoint->type) {
	case BKPT_SOFT:
		{
			/**
			@todo check RVC for size/alignment
			*/
			if (!(breakpoint->length == 4 || breakpoint->length == 2)) {
				LOG_ERROR("%s: Invalid breakpoint length %d",
					target_name(target), breakpoint->length);
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}

			if (0 != (breakpoint->address % 2)) {
				LOG_ERROR("%s: Invalid breakpoint alignment for address 0x%" TARGET_PRIxADDR,
					target_name(target), breakpoint->address);
				return ERROR_TARGET_UNALIGNED_ACCESS;
			}

			{
				int const error_code =
					target_read_memory(target, breakpoint->address, 2, breakpoint->length / 2, breakpoint->orig_instr);

				if (ERROR_OK != error_code) {
					LOG_ERROR("%s: Failed to read original instruction at 0x%" TARGET_PRIxADDR,
						target_name(target),
						breakpoint->address);
					return error_code;
				}
			}

			uint8_t buff[4];
			buf_set_u32(buff, 0, breakpoint->length * CHAR_BIT, breakpoint->length == 4 ? ebreak() : ebreak_c());

			{
				int const error_code =
					target_write_memory(target, breakpoint->address, 2, breakpoint->length / 2, buff);

				if (ERROR_OK != error_code) {
					LOG_ERROR("%s: Failed to write %d-byte breakpoint instruction at 0x%" TARGET_PRIxADDR,
						target_name(target), breakpoint->length, breakpoint->address);
					return error_code;
				}
			}
		}
		break;

	case BKPT_HARD:
		{
			struct trigger trigger;
			trigger_from_breakpoint(&trigger, breakpoint);
			{
				int const error_code =
					add_trigger(target, &trigger);

				if (ERROR_OK != error_code)
					return error_code;
			}
		}
		break;

	default:
		LOG_INFO("%s: OpenOCD only supports hardware and software breakpoints.",
			target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = true;
	return ERROR_OK;
}

/** @return error code */
static int
remove_trigger(struct target *const target,
	struct trigger *const trigger)
{
	{
		int const error_code = riscv_enumerate_triggers(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	int first_hart = -1;

	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		if (first_hart < 0) {
			first_hart = hartid;
			break;
		}
	}

	assert(first_hart >= 0);
	assert(trigger);
	riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);

	unsigned i;

	for (i = 0; i < rvi->harts[first_hart].trigger_count; ++i) {
		if (rvi->trigger_unique_id[i] == trigger->unique_id)
			break;
	}

	if (rvi->harts[first_hart].trigger_count <= i) {
		LOG_ERROR("%s: Couldn't find the hardware resources used by hardware trigger.",
			target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	LOG_DEBUG("%s: Stop using resource %d for bp %d", target_name(target), i, trigger->unique_id);

	for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		riscv_reg_t tselect;
		{
			int const error_code =
				riscv_get_register_on_hart(target, &tselect, hartid, GDB_REGNO_TSELECT);

			if (ERROR_OK != error_code)
				return error_code;
		}

		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, i);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect);
	}

	rvi->trigger_unique_id[i] = -1;

	return ERROR_OK;
}

/** @return error code */
static int
riscv_remove_breakpoint(struct target *const target,
	struct breakpoint *breakpoint)
{
	assert(breakpoint);

	switch (breakpoint->type) {
	case BKPT_SOFT:
		{
			int const error_code =
				target_write_memory(target, breakpoint->address, 2, breakpoint->length / 2, breakpoint->orig_instr);

			if (ERROR_OK != error_code) {
				LOG_ERROR("%s: Failed to restore instruction for %d-byte breakpoint at "
					"0x%" TARGET_PRIxADDR,
					target_name(target),
					breakpoint->length,
					breakpoint->address);
				return error_code;
			}
		}
		break;

	case BKPT_HARD:
		{
			struct trigger trigger;
			trigger_from_breakpoint(&trigger, breakpoint);
			int const error_code = remove_trigger(target, &trigger);

			if (ERROR_OK != error_code)
				return error_code;

		}
		break;

	default:
		LOG_INFO("%s: OpenOCD only supports hardware and software breakpoints.",
			target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = false;

	return ERROR_OK;
}

/** @return error code */
static void
trigger_from_watchpoint(struct trigger *const trigger,
	struct watchpoint const *const watchpoint)
{
	assert(trigger);

	trigger->address = watchpoint->address;
	trigger->length = watchpoint->length;
	trigger->mask = watchpoint->mask;
	trigger->value = watchpoint->value;
	trigger->read = (watchpoint->rw == WPT_READ || watchpoint->rw == WPT_ACCESS);
	trigger->write = (watchpoint->rw == WPT_WRITE || watchpoint->rw == WPT_ACCESS);
	trigger->execute = false;
	/* unique_id is unique across both breakpoints and watchpoints. */
	assert(watchpoint);
	trigger->unique_id = watchpoint->unique_id;
}

/** @return error code */
static int
riscv_add_watchpoint(struct target *const target,
	struct watchpoint *const watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	{
		int const error_code = add_trigger(target, &trigger);

		if (ERROR_OK != error_code)
			return error_code;
	}

	assert(watchpoint);
	watchpoint->set = true;

	return ERROR_OK;
}

/** @return error code */
static int
riscv_remove_watchpoint(struct target *const target,
	struct watchpoint *const watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	{
		int const error_code = remove_trigger(target, &trigger);

		if (ERROR_OK != error_code)
			return error_code;
	}

	assert(watchpoint);
	watchpoint->set = false;

	return ERROR_OK;
}

/** Sets @c hit_watchpoint to the first watchpoint identified as causing the
current halt.

The GDB server uses this information to tell GDB what data address has
been hit, which enables GDB to print the hit variable along with its old
and new value.
*/
/** @return error code */
static int
riscv_hit_watchpoint(struct target *const target,
	struct watchpoint **const hit_watchpoint)
{
	assert(target);
	struct watchpoint *wp = target->watchpoints;

	LOG_DEBUG("%s: Current hartid = %d",
			target_name(target),
			riscv_current_hartid(target));

	/**
	@todo instead of disassembling the instruction that we think caused the trigger,
	check the hit bit of each watchpoint first.
	The hit bit is simpler and more reliable to check
	but as it is optional and relatively new, not all hardware will implement it
	*/
	riscv_reg_t dpc;
	riscv_get_register(target, &dpc, GDB_REGNO_DPC);
	const uint8_t length = 4;
	LOG_DEBUG("%s: dpc is 0x%" PRIx64,
			target_name(target), dpc);

	/* fetch the instruction at dpc */
	uint8_t buffer[length];

	{
		int const error_code =target_read_buffer(target, dpc, length, buffer);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: Failed to read instruction at dpc 0x%" PRIx64,
					target_name(target),
					dpc);
			return error_code;
		}
	}

	uint32_t instruction = 0;

	for (int i = 0; i < length; ++i) {
		LOG_DEBUG("%s: Next byte is %x",
				target_name(target),
				buffer[i]);
		instruction += (buffer[i] << 8 * i);

	}
	LOG_DEBUG("%s: Full instruction is %x",
			target_name(target),
			instruction);

	/* find out which memory address is accessed by the instruction at dpc */
	/* opcode is first 7 bits of the instruction */
	uint8_t opcode = instruction & 0x7F;
	uint32_t rs1;
	int16_t imm;
	riscv_reg_t mem_addr;

	if (opcode == MATCH_LB || opcode == MATCH_SB) {
		rs1 = (instruction & 0xf8000) >> 15;
		riscv_get_register(target, &mem_addr, rs1);

		if (opcode == MATCH_SB) {
			LOG_DEBUG("%s: %x is store instruction",
					target_name(target),
					instruction);
			imm = ((instruction & 0xf80) >> 7) | ((instruction & 0xfe000000) >> 20);
		} else {
			LOG_DEBUG("%s: %x is load instruction",
					target_name(target),
					instruction);
			imm = (instruction & 0xfff00000) >> 20;
		}
		/* sign extend 12-bit imm to 16-bits */
		if (imm & (1 << 11))
			imm |= 0xf000;
		mem_addr += imm;
		LOG_DEBUG("%s: memory address=0x%" PRIx64,
				target_name(target),
				mem_addr);
	} else {
		LOG_DEBUG("%s: %x is not a RV32I load or store",
				target_name(target), instruction);
		return ERROR_TARGET_INVALID;
	}

	for (; wp; wp = wp->next) {
		/**
		@todo support length/mask
		*/
		if (wp->address == mem_addr) {
			assert(hit_watchpoint);
			*hit_watchpoint = wp;
			LOG_DEBUG("%s: Hit address=%" TARGET_PRIxADDR,
					target_name(target), wp->address);
			return ERROR_OK;
		}
	}

	/* No match found - either we hit a watchpoint caused by an instruction that
	 * this function does not yet disassemble, or we hit a breakpoint.
	 *
	 * OpenOCD will behave as if this function had never been implemented i.e.
	 * report the halt to GDB with no address information. */
	return ERROR_TARGET_INVALID;
}

/** @return error code */
static int
riscv_examine(struct target *const target)
{
	assert(target);
	LOG_DEBUG("%s: riscv_examine()", target_name(target));

	if (target_was_examined(target)) {
		LOG_DEBUG("%s: Target was already examined.", target_name(target));
		return ERROR_OK;
	}

	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */
	uint32_t dtmcontrol;
	{
		int const error_code = dtmcontrol_scan(target->tap, 0, &dtmcontrol);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: fatal: examine failure, JTAG/TAP error",
				target_name(target));
			return error_code;
		}
	}

	LOG_DEBUG("%s: dtmcontrol=0x%x", target_name(target), dtmcontrol);
	riscv_info_t *const info = target->arch_info;
	info->dtm_version = FIELD_GET(dtmcontrol, DTMCONTROL_VERSION);
	LOG_DEBUG("%s:  version=0x%x", target_name(target), info->dtm_version);

	{
		int const error_code = riscv_013_init_target(info->cmd_ctx, target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	return riscv_013_examine(target);
}

/** @return error code */
static int
riscv_select_current_hart(struct target *const target)
{
	if (riscv_rtos_enabled(target)) {
		riscv_info_t *const rvi = riscv_info(target);
		assert(rvi);

		if (rvi->rtos_hartid == -1)
			rvi->rtos_hartid = target->rtos->current_threadid - 1;

		return riscv_set_current_hartid(target, rvi->rtos_hartid);
	} else
		return riscv_set_current_hartid(target, target->coreid);
}

static inline bool
is_valid_size_and_alignment(target_addr_t const address,
	uint32_t const size)
{
	return
		(1u == size || 2u == size || 4u == size || 8u == size || 16u == size) &&
		0 == (address % size);
}

/** @return error code */
static int
riscv_read_memory(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint32_t const count,
	uint8_t *const buffer)
{
	assert(target);

	if (!is_valid_size_and_alignment(address, size)) {
		LOG_ERROR("%s: Invalid size/alignment: address=0x%" TARGET_PRIxADDR ", size=%d",
				target_name(target), address, size);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (0 == count)
		return ERROR_OK;

	assert(buffer);

	{
		int const error_code = riscv_select_current_hart(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	return riscv_013_read_memory(target, address, size, count, buffer);
}

static int
riscv_write_memory(struct target *const target,
	target_addr_t const address,
	uint32_t const size,
	uint32_t const count,
	uint8_t const *const buffer)
{
	assert(target);

	if (!is_valid_size_and_alignment(address, size)) {
		LOG_ERROR("%s: Invalid size/alignment: address=0x%" TARGET_PRIxADDR ", size=%d",
				target_name(target), address, size);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (0 == count)
		return ERROR_OK;

	{
		int const error_code = riscv_select_current_hart(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	return riscv_013_write_memory(target, address, size, count, buffer);
}

/** @return error code */
static int
riscv_get_gdb_reg_list(struct target *const target,
	struct reg **reg_list[],
	int *const reg_list_size,
	enum target_register_class const reg_class)
{
	assert(target);
	LOG_DEBUG("%s: reg_class=%d",
		target_name(target), reg_class);

	riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);

	LOG_DEBUG("%s: rtos_hartid=%d current_hartid=%d",
			target_name(target), rvi->rtos_hartid, rvi->current_hartid);

	if (!target->reg_cache) {
		LOG_ERROR("%s: Target not initialized.",
				target_name(target));
		return ERROR_TARGET_INIT_FAILED;
	}

	{
		int const error_code = riscv_select_current_hart(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	switch (reg_class) {
		case REG_CLASS_GENERAL:
			assert(reg_list_size);
			*reg_list_size = 32;
			break;

		case REG_CLASS_ALL:
			assert(reg_list_size);
			*reg_list_size = target->reg_cache->num_regs;
			break;

		default:
			LOG_ERROR("%s: Unsupported reg_class: %d",
				target_name(target), reg_class);
			return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	*reg_list = calloc(*reg_list_size, sizeof(struct reg *));
	if (!*reg_list) {
		LOG_ERROR("%s: Fatal: No free memory!", target_name(target));
		return ERROR_FAIL;
	}

	for (int i = 0; i < *reg_list_size; ++i) {
		assert(!target->reg_cache->reg_list[i].valid || target->reg_cache->reg_list[i].size > 0);
		(*reg_list)[i] = &target->reg_cache->reg_list[i];
	}

	return ERROR_OK;
}

enum riscv_poll_hart_e {
	RPH_NO_CHANGE,
	RPH_DISCOVERED_HALTED,
	RPH_DISCOVERED_RUNNING,
	RPH_ERROR
};

static enum riscv_poll_hart_e
riscv_poll_hart(struct target *const target,
	int const hartid)
{
	if (ERROR_OK != riscv_set_current_hartid(target, hartid))
		return RPH_ERROR;

	LOG_DEBUG("%s: polling hart %d, target->state=%d", target_name(target), hartid, target->state);

	/* If OpenOCD thinks we're running but this hart is halted then it's time
	 * to raise an event. */
	bool const halted = riscv_is_halted(target);

	if (TARGET_HALTED != target->state && halted) {
		LOG_DEBUG("%s:  triggered a halt",
			target_name(target));
		return RPH_DISCOVERED_HALTED;
	} else if (target->state != TARGET_RUNNING && !halted) {
		LOG_DEBUG("%s:  triggered running", target_name(target));
		target->state = TARGET_RUNNING;
		return RPH_DISCOVERED_RUNNING;
	}

	return RPH_NO_CHANGE;
}

static int
riscv_openocd_poll(struct target *const target)
{
	assert(target);
	LOG_DEBUG("%s: polling all harts", target_name(target));
	int halted_hart = -1;

	if (riscv_rtos_enabled(target)) {
		/* Check every hart for an event. */
		for (int i = 0; i < riscv_count_harts(target); ++i) {
			enum riscv_poll_hart_e const poll_result =
				riscv_poll_hart(target, i);

			switch (poll_result) {
			case RPH_NO_CHANGE:
			case RPH_DISCOVERED_RUNNING:
				continue;

			case RPH_DISCOVERED_HALTED:
				halted_hart = i;
				break;

			case RPH_ERROR:
				LOG_ERROR("%s: poll HART error", target_name(target));
				return ERROR_TARGET_FAILURE;
			}
		}

		if (halted_hart == -1) {
			LOG_DEBUG("%s:  no harts just halted, target->state=%d",
				target_name(target), target->state);
			return ERROR_OK;
		}

		LOG_DEBUG("%s:  hart %d halted", target_name(target), halted_hart);

		/* If we're here then at least one hart triggered.  That means
		 * we want to go and halt _every_ hart in the system, as that's
		 * the invariant we hold here.	Some harts might have already
		 * halted (as we're either in single-step mode or they also
		 * triggered a breakpoint), so don't attempt to halt those
		 * harts. */
		for (int i = 0; i < riscv_count_harts(target); ++i)
			riscv_halt_one_hart(target, i);

	} else {
		enum riscv_poll_hart_e const poll_event =
			riscv_poll_hart(target, riscv_current_hartid(target));

		if (RPH_NO_CHANGE == poll_event || RPH_DISCOVERED_RUNNING == poll_event)
			return ERROR_OK;
		else if (RPH_ERROR == poll_event) {
			LOG_ERROR("%s: poll HART error", target_name(target));
			return ERROR_TARGET_FAILURE;
		}

		halted_hart = riscv_current_hartid(target);
		LOG_DEBUG("%s:  hart %d halted", target_name(target), halted_hart);
	}

	target->state = TARGET_HALTED;

	switch (riscv_halt_reason(target, halted_hart)) {
	case RISCV_HALT_BREAKPOINT:
		target->debug_reason = DBG_REASON_BREAKPOINT;
		break;

	case RISCV_HALT_TRIGGER:
		target->debug_reason = DBG_REASON_WATCHPOINT;
		break;

	case RISCV_HALT_INTERRUPT:
		target->debug_reason = DBG_REASON_DBGRQ;
		break;

	case RISCV_HALT_SINGLESTEP:
		target->debug_reason = DBG_REASON_SINGLESTEP;
		break;

	case RISCV_HALT_UNKNOWN:
		target->debug_reason = DBG_REASON_UNDEFINED;
		break;

	case RISCV_HALT_ERROR:
		LOG_ERROR("%s: halt error", target_name(target));
		return ERROR_TARGET_FAILURE;

	/**
	@bug no default case
	*/
	}

	if (riscv_rtos_enabled(target)) {
		target->rtos->current_threadid = halted_hart + 1;
		target->rtos->current_thread = halted_hart + 1;
		riscv_set_rtos_hartid(target, halted_hart);
	}

	target->state = TARGET_HALTED;

	if (target->debug_reason == DBG_REASON_BREAKPOINT) {
		int retval;
		if (0 != riscv_semihosting(target, &retval))
			return retval;
	}

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return ERROR_OK;
}

static int
riscv_openocd_halt(struct target *const target)
{
	assert(target);
	LOG_DEBUG("%s: halting all harts", target_name(target));

	{
		int const error_code = riscv_halt_all_harts(target);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: Unable to halt all harts", target_name(target));
			return error_code;
		}
	}

	register_cache_invalidate(target->reg_cache);

	if (riscv_rtos_enabled(target)) {
		riscv_info_t const *const rvi = riscv_info(target);
		assert(rvi);

		if (rvi->rtos_hartid != -1) {
			LOG_DEBUG("%s: halt requested on RTOS hartid %d",
				target_name(target), rvi->rtos_hartid);
			assert(target->rtos);
			target->rtos->current_threadid = rvi->rtos_hartid + 1;
			target->rtos->current_thread = rvi->rtos_hartid + 1;
		} else
			LOG_DEBUG("%s: halt requested, but no known RTOS hartid",
				target_name(target));
	}

	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_DBGRQ;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return ERROR_OK;
}

int
riscv_step_rtos_hart(struct target *const target)
{
	riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);
	int hartid = rvi->current_hartid;

	if (riscv_rtos_enabled(target)) {
		hartid = rvi->rtos_hartid;
		if (hartid == -1) {
			LOG_DEBUG("%s: GDB has asked me to step \"any\" thread, so I'm stepping hart 0.", target_name(target));
			hartid = 0;
		}
	}

	{
		int const error_code = riscv_set_current_hartid(target, hartid);

		if (ERROR_OK != error_code)
			return error_code;
	}

	LOG_DEBUG("%s: stepping hart %d", target_name(target), hartid);

	if (!riscv_is_halted(target)) {
		LOG_ERROR("%s: Hart isn't halted before single step!",
			target_name(target));
		return ERROR_TARGET_NOT_HALTED;
	}

	riscv_invalidate_register_cache(target);
	riscv_013_on_step(target);

	{
		int const error_code = riscv_013_step_current_hart(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	riscv_invalidate_register_cache(target);

	if (!riscv_is_halted(target)) {
		LOG_ERROR("%s: Hart was not halted after single step!", target_name(target));
		return ERROR_TARGET_NOT_HALTED;
	}

	return ERROR_OK;
}

static int
riscv_openocd_resume(struct target *const target,
	int const current,
	target_addr_t const address,
	int const handle_breakpoints,
	int const debug_execution)
{
	assert(target);
	LOG_DEBUG("%s: handle_breakpoints=%d  debug_reason=%d",
		target_name(target),
		handle_breakpoints,
		target->debug_reason);

	if (!current)
		riscv_set_register(target, GDB_REGNO_PC, address);

	if (target->debug_reason == DBG_REASON_WATCHPOINT) {
		/*
			To be able to run off a trigger,
			disable all the triggers,
			step, and then resume as usual.
		*/
		struct watchpoint *watchpoint = target->watchpoints;
		bool trigger_temporarily_cleared[RISCV_MAX_HWBPS] = {0};

		int error_code = ERROR_OK;

		for (int i = 0; watchpoint && ERROR_OK == error_code; ++i) {
			LOG_DEBUG("%s: watchpoint %d: set=%d", target_name(target), i, watchpoint->set);
			trigger_temporarily_cleared[i] = watchpoint->set;

			if (watchpoint->set)
				error_code = target_remove_watchpoint(target, watchpoint);

			watchpoint = watchpoint->next;
		}

		if (ERROR_OK == error_code)
			error_code = riscv_step_rtos_hart(target);

		{
			int i = 0;
			for (watchpoint = target->watchpoints; watchpoint; watchpoint = watchpoint->next, ++i) {
				LOG_DEBUG("%s: watchpoint %d: cleared=%d",
					target_name(target),
					i,
					trigger_temporarily_cleared[i]);

				if (trigger_temporarily_cleared[i]) {
					int const err = target_add_watchpoint(target, watchpoint);
					error_code = ERROR_OK == error_code ? err : error_code;
				}
			}
		}

		if (ERROR_OK != error_code)
			return error_code;
	}

	{
		int const error_code = riscv_resume_all_harts(target);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: unable to resume all harts", target_name(target));
			return error_code;
		}
	}

	register_cache_invalidate(target->reg_cache);
	target->state = TARGET_RUNNING;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	return ERROR_OK;
}

static int
riscv_openocd_step(struct target *const target,
	int const current,
	target_addr_t const address,
	int const handle_breakpoints)
{
	LOG_DEBUG("%s: handle_breakpoints=%d",
		target_name(target), handle_breakpoints);

	if (!current) {
		int const error_code = riscv_set_register(target, GDB_REGNO_PC, address);

		if (ERROR_OK != error_code)
			return error_code;
	}

	{
		int const error_code = riscv_step_rtos_hart(target);

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: unable to step rtos hart", target_name(target));
			return error_code;
		}
	}

	register_cache_invalidate(target->reg_cache);
	target->state = TARGET_RUNNING;
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_SINGLESTEP;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return ERROR_OK;
}

/* Command Handlers */
COMMAND_HANDLER(riscv_set_command_timeout_sec)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	int const timeout = atoi(CMD_ARGV[0]);

	if (timeout <= 0) {
		LOG_ERROR("%s is not a valid integer argument for command.", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_UNDERFLOW;
	}

	riscv_command_timeout_sec = timeout;
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_reset_timeout_sec)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	int const timeout = atoi(CMD_ARGV[0]);

	if (timeout <= 0) {
		LOG_ERROR("%s is not a valid integer argument for command.", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_UNDERFLOW;
	}

	riscv_reset_timeout_sec = timeout;
	return ERROR_OK;
}

COMMAND_HANDLER(riscv_test_compliance)
{
	struct target *const target = get_current_target(CMD_CTX);

	if (CMD_ARGC > 0) {
		LOG_ERROR("%s: Command does not take any parameters.", target_name(target));
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return riscv_013_test_compliance(target);
}

COMMAND_HANDLER(riscv_set_prefer_sba)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], riscv_prefer_sba);
	return ERROR_OK;
}

static int
parse_error(char const *const string,
	char const c,
	unsigned const position)
{
	/**
	@bug Non-portable language extension: array with dynamic size
	*/
	char buf[position + 2];
	memset(buf, ' ', position);
	buf[position] = '^';
	buf[position + 1] = 0;

	LOG_ERROR("Parse error at character %c in:" "\n" "%s" "\n" "%s",
			c, string, buf);
	return ERROR_COMMAND_SYNTAX_ERROR;
}

static int
parse_ranges(range_t **const ranges,
	char const **const argv)
{
	for (unsigned pass = 0; pass < 2; ++pass) {
		unsigned range = 0;
		unsigned low = 0;
		bool parse_low = true;
		unsigned high = 0;

		for (unsigned i = 0; i == 0 || argv[0][i-1]; ++i) {
			char c = argv[0][i];

			if (isspace(c)) {
				/* Ignore whitespace. */
				continue;
			}

			if (parse_low) {
				if (isdigit(c)) {
					low *= 10;
					low += c - '0';
				} else if (c == '-') {
					parse_low = false;
				} else if (c == ',' || c == 0) {
					if (pass == 1) {
						(*ranges)[range].low = low;
						(*ranges)[range].high = low;
					}
					low = 0;
					++range;
				} else {
					return parse_error(argv[0], c, i);;
				}
			} else {
				if (isdigit(c)) {
					high *= 10;
					high += c - '0';
				} else if (c == ',' || c == 0) {
					parse_low = true;
					if (pass == 1) {
						(*ranges)[range].low = low;
						(*ranges)[range].high = high;
					}
					low = 0;
					high = 0;
					++range;
				} else {
					return parse_error(argv[0], c, i);
				}
			}
		}

		if (pass == 0) {
			if (*ranges)
				free(*ranges);
			/**
			@todo check for free
			*/
			*ranges = calloc(range + 2, sizeof(range_t));
			assert(*ranges);
		} else {
			(*ranges)[range].low = 1;
			(*ranges)[range].high = 0;
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(riscv_set_expose_csrs)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *const target = get_current_target(CMD_CTX);
	assert(target);

	return parse_ranges(&expose_csr, CMD_ARGV);
}

COMMAND_HANDLER(riscv_set_expose_custom)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return parse_ranges(&expose_custom, CMD_ARGV);
}

COMMAND_HANDLER(riscv_authdata_read)
{
	if (CMD_ARGC != 0) {
		LOG_ERROR("Command takes no parameters");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *const target = get_current_target(CMD_CTX);

	if (!target) {
		LOG_ERROR("target is NULL!");
		return ERROR_TARGET_INVALID;
	}

	riscv_info_t const *const rvi = riscv_info(target);

	if (!rvi) {
		LOG_ERROR("%s: riscv_info is NULL!",
			target_name(target));
		return ERROR_TARGET_INVALID;
	}

	{
		uint32_t value;

		{
			int const error_code = riscv_013_authdata_read(target, &value);

			if (ERROR_OK != error_code)
				return error_code;
		}

		command_print(CMD_CTX, "0x%" PRIx32, value);
		return ERROR_OK;
	}
}

COMMAND_HANDLER(riscv_authdata_write)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes exactly 1 argument");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *const target = get_current_target(CMD_CTX);

	uint32_t value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], value);

	return riscv_013_authdata_write(target, value);
}

COMMAND_HANDLER(riscv_dmi_read)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Command takes 1 parameter");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct target *const target = get_current_target(CMD_CTX);

	if (!target) {
		LOG_ERROR("target is NULL!");
		return ERROR_TARGET_INVALID;
	}

	riscv_info_t const *const rvi = riscv_info(target);

	if (!rvi) {
		LOG_ERROR("%s: riscv_info is NULL!", target_name(target));
		return ERROR_TARGET_INVALID;
	}

	{
		uint32_t address;
		uint32_t value;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

		{
			int const error_code = dmi_read(target, &value, address);

			if (ERROR_OK != error_code)
				return error_code;
		}

		command_print(CMD_CTX, "0x%" PRIx32, value);
		return ERROR_OK;
	}
}


COMMAND_HANDLER(riscv_dmi_write)
{
	if (CMD_ARGC != 2) {
		LOG_ERROR("Command takes exactly 2 arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint32_t address;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

	uint32_t value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	struct target *const target = get_current_target(CMD_CTX);
	return dmi_write(target, address, value);
}

COMMAND_HANDLER(riscv_test_sba_config_reg)
{
	if (CMD_ARGC != 4) {
		LOG_ERROR("Command takes exactly 4 arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	target_addr_t legal_address;
	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[0], legal_address);

	uint32_t num_words;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], num_words);

	target_addr_t illegal_address;
	COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[2], illegal_address);

	bool run_sbbusyerror_test;
	COMMAND_PARSE_ON_OFF(CMD_ARGV[3], run_sbbusyerror_test);

	struct target *const target = get_current_target(CMD_CTX);
	riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);

	return
		riscv_013_test_sba_config_reg(target, legal_address, num_words, illegal_address, run_sbbusyerror_test);
}

static struct command_registration const riscv_exec_command_handlers[] = {
	{
		.name = "test_compliance",
		.handler = riscv_test_compliance,
		.mode = COMMAND_EXEC,
		.usage = "riscv test_compliance",
		.help = "Runs a basic compliance test suite against the RISC-V Debug Spec."
	},
	{
		.name = "set_command_timeout_sec",
		.handler = riscv_set_command_timeout_sec,
		.mode = COMMAND_ANY,
		.usage = "riscv set_command_timeout_sec [sec]",
		.help = "Set the wall-clock timeout (in seconds) for individual commands"
	},
	{
		.name = "set_reset_timeout_sec",
		.handler = riscv_set_reset_timeout_sec,
		.mode = COMMAND_ANY,
		.usage = "riscv set_reset_timeout_sec [sec]",
		.help = "Set the wall-clock timeout (in seconds) after reset is deasserted"
	},
	{
		.name = "set_prefer_sba",
		.handler = riscv_set_prefer_sba,
		.mode = COMMAND_ANY,
		.usage = "riscv set_prefer_sba on|off",
		.help = "When on, prefer to use System Bus Access to access memory. "
			"When off, prefer to use the Program Buffer to access memory."
	},
	{
		.name = "expose_csrs",
		.handler = riscv_set_expose_csrs,
		.mode = COMMAND_ANY,
		.usage = "riscv expose_csrs n0[-m0][,n1[-m1]]...",
		.help = "Configure a list of inclusive ranges for CSRs to expose in "
				"addition to the standard ones. This must be executed before "
				"`init`."
	},
	{
		.name = "expose_custom",
		.handler = riscv_set_expose_custom,
		.mode = COMMAND_ANY,
		.usage = "riscv expose_custom n0[-m0][,n1[-m1]]...",
		.help = "Configure a list of inclusive ranges for custom registers to "
			"expose. custom0 is accessed as abstract register number 0xc000, "
			"etc. This must be executed before `init`."
	},
	{
		.name = "authdata_read",
		.handler = riscv_authdata_read,
		.mode = COMMAND_ANY,
		.usage = "riscv authdata_read",
		.help = "Return the 32-bit value read from authdata."
	},
	{
		.name = "authdata_write",
		.handler = riscv_authdata_write,
		.mode = COMMAND_ANY,
		.usage = "riscv authdata_write value",
		.help = "Write the 32-bit value to authdata."
	},
	{
		.name = "dmi_read",
		.handler = riscv_dmi_read,
		.mode = COMMAND_ANY,
		.usage = "riscv dmi_read address",
		.help = "Perform a 32-bit DMI read at address, returning the value."
	},
	{
		.name = "dmi_write",
		.handler = riscv_dmi_write,
		.mode = COMMAND_ANY,
		.usage = "riscv dmi_write address value",
		.help = "Perform a 32-bit DMI write of value at address."
	},
	{
		.name = "test_sba_config_reg",
		.handler = riscv_test_sba_config_reg,
		.mode = COMMAND_ANY,
		.usage = "riscv test_sba_config_reg legal_address num_words"
			"illegal_address run_sbbusyerror_test[on/off]",
		.help = "Perform a series of tests on the SBCS register."
			"Inputs are a legal, 128-byte aligned address and a number of words to"
			"read/write starting at that address (i.e., address range [legal address,"
			"legal_address+word_size*num_words) must be legally readable/writable)"
			", an illegal, 128-byte aligned address for error flag/handling cases,"
			"and whether sbbusyerror test should be run."
	},
	COMMAND_REGISTRATION_DONE
};

/**
 * To be noted that RISC-V targets use the same semihosting commands as
 * ARM targets.
 *
 * The main reason is compatibility with existing tools. For example the
 * Eclipse OpenOCD/SEGGER J-Link/QEMU plug-ins have several widgets to
 * configure semihosting, which generate commands like `arm semihosting
 * enable`.
 * A secondary reason is the fact that the protocol used is exactly the
 * one specified by ARM. If RISC-V will ever define its own semihosting
 * protocol, then a command like `riscv semihosting enable` will make
 * sense, but for now all semihosting commands are prefixed with `arm`.
 */
static struct command_registration const arm_exec_command_handlers[] = {
	{
		"semihosting",
		.handler = handle_common_semihosting_command,
		.mode = COMMAND_EXEC,
		.usage = "['enable'|'disable']",
		.help = "activate support for semihosting operations",
	},
	{
		"semihosting_cmdline",
		.handler = handle_common_semihosting_cmdline,
		.mode = COMMAND_EXEC,
		.usage = "arguments",
		.help = "command line arguments to be passed to program",
	},
	{
		"semihosting_fileio",
		.handler = handle_common_semihosting_fileio_command,
		.mode = COMMAND_EXEC,
		.usage = "['enable'|'disable']",
		.help = "activate support for semihosting fileio operations",
	},
	{
		"semihosting_resexit",
		.handler = handle_common_semihosting_resumable_exit_command,
		.mode = COMMAND_EXEC,
		.usage = "['enable'|'disable']",
		.help = "activate support for semihosting resumable exit",
	},
	COMMAND_REGISTRATION_DONE
};

struct command_registration const riscv_command_handlers[] = {
	{
		.name = "riscv",
		.mode = COMMAND_ANY,
		.help = "RISC-V Command Group",
		.usage = "",
		.chain = riscv_exec_command_handlers
	},
	{
		.name = "arm",
		.mode = COMMAND_ANY,
		.help = "ARM Command Group",
		.usage = "",
		.chain = arm_exec_command_handlers
	},
	COMMAND_REGISTRATION_DONE
};

/**
	@return number of HARTs
	@bug return signed value
*/
int
riscv_count_harts(struct target const *const target)
{
	/**
	@bug riscv_count_harts 1 for NULL and bad target
	*/
	if (target == NULL)
		return 1;

	riscv_info_t const *const rvi = riscv_info(target);

	if (!rvi)
		return 1;

	return rvi->hart_count;
}

/**
	This function is called when the debug user wants to change the value of a
	register. The new value may be cached, and may not be written until the hart
	is resumed.

	@return error code
 */
int
riscv_set_register(struct target *const target,
	enum gdb_riscv_regno const gdb_reg_no,
	riscv_reg_t const value)
{
	return riscv_set_register_on_hart(target, riscv_current_hartid(target), gdb_reg_no, value);
}

/**	@note Syntactical sugar
	@return error code
*/
int
riscv_get_register(struct target *const target,
	riscv_reg_t *const value,
	enum gdb_riscv_regno const r)
{
	return
		riscv_get_register_on_hart(target, value, riscv_current_hartid(target), r);
}

/** @return error code*/
int
riscv_get_register_on_hart(struct target *const target,
	riscv_reg_t *const value/**<[out]*/,
	int const hartid,
	enum gdb_riscv_regno const regid)
{
	if (riscv_current_hartid(target) != hartid)
		riscv_invalidate_register_cache(target);

	int const err = riscv_013_get_register(target, value, hartid, regid);

	if (riscv_current_hartid(target) != hartid)
		riscv_invalidate_register_cache(target);

	assert(value);
	LOG_DEBUG("%s: [%d] %s: %" PRIx64, target_name(target), hartid, gdb_regno_name(regid), *value);
	return err;
}

/** @return error code*/
static int
register_get(struct reg *const reg)
{
	assert(reg);
	riscv_reg_info_t *const reg_info = reg->arch_info;
	assert(reg_info);
	struct target *const target = reg_info->target;
	uint64_t value;
	{
		int const error_code = riscv_get_register(target, &value, reg->number);

		if (ERROR_OK != error_code)
			return error_code;
	}

	buf_set_u64(reg->value, 0, reg->size, value);
	return ERROR_OK;
}

/** @return error code*/
static int
register_set(struct reg *const reg,
	uint8_t *const buf)
{
	assert(reg);
	assert(buf);
	uint64_t const value = buf_get_u64(buf, 0, reg->size);

	riscv_reg_info_t *const reg_info = reg->arch_info;
	struct target *const target = reg_info->target;
	assert(target);
	LOG_DEBUG("%s: write 0x%" PRIx64 " to %s", target_name(target), value, reg->name);
	struct reg *r = &target->reg_cache->reg_list[reg->number];
	assert(r);
	r->valid = true;
	assert(r->value);
	memcpy(r->value, buf, (r->size + 7) / 8);
	return riscv_set_register(target, reg->number, value);
}

static struct reg_arch_type const riscv_reg_arch_type = {
	.get = register_get,
	.set = register_set
};

struct target_type riscv_target = {
	.name = "riscv",
	.poll = riscv_openocd_poll,
	.arch_state = riscv_013_arch_state,
#if 0
	.target_request_data = NULL,
#endif
	.halt = riscv_openocd_halt,
	.resume = riscv_openocd_resume,
	.step = riscv_openocd_step,
	.assert_reset = riscv_013_assert_reset,
	.deassert_reset = riscv_013_deassert_reset,
#if 0
	.soft_reset_halt = NULL,
#endif
	.get_gdb_reg_list = riscv_get_gdb_reg_list,
	.read_memory = riscv_read_memory,
	.write_memory = riscv_write_memory,
#if 0
    .read_buffer = NULL,
	.write_buffer = NULL,
	.checksum_memory = riscv_checksum_memory,
	.blank_check_memory = NULL,
#endif
	.add_breakpoint = riscv_add_breakpoint,
#if 0
	.add_context_breakpoint = NULL,
	.add_hybrid_breakpoint = NULL,
#endif
	.remove_breakpoint = riscv_remove_breakpoint,
	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,
	.hit_watchpoint = riscv_hit_watchpoint,
#if 0
	.run_algorithm = riscv_run_algorithm,
	.start_algorithm = NULL,
	.wait_algorithm = NULL,
#endif
	.commands = riscv_command_handlers,
#if 0
	.target_create = NULL,
	.target_jim_configure = NULL,
	.target_jim_commands = NULL,
#endif
	.examine = riscv_examine,
	.init_target = riscv_init_target,
	.deinit_target = riscv_deinit_target,
#if 0
	.virt2phys = NULL,
	.read_phys_memory = NULL,
	.write_phys_memory = NULL,
	.mmu = NULL,
	.check_reset = NULL,
	.get_gdb_fileio_info = NULL,
	.gdb_fileio_end = NULL,
	.profiling = NULL,
#endif
};
