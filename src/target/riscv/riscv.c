#include "riscv.h"
#include "opcodes.h"
#include "debug_defines.h"

#include "target/algorithm.h"
#include "target/target_type.h"
#include "jtag/jtag.h"
#include "target/register.h"
#include "target/breakpoints.h"
#include "rtos/rtos.h"

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

/** @name JTAG registers. */
/**@{*/
#define DTMCONTROL_VERSION			(0xF)
/**@}*/

/** @name External handlers

	@todo place in header
*/
/**@{*/
__COMMAND_HANDLER(handle_common_semihosting_command);
__COMMAND_HANDLER(handle_common_semihosting_fileio_command);
__COMMAND_HANDLER(handle_common_semihosting_resumable_exit_command);
__COMMAND_HANDLER(handle_common_semihosting_cmdline);
/**@}*/

struct range_s {
	uint16_t low;
	uint16_t high;
};
typedef struct range_s range_t;

int riscv_command_timeout_sec = DEFAULT_COMMAND_TIMEOUT_SEC;
int riscv_reset_timeout_sec = DEFAULT_RESET_TIMEOUT_SEC;

bool riscv_prefer_sba = false;

/** In addition to the ones in the standard spec, we'll also expose additional CSRs in this list.

The list is either NULL, or a series of ranges (inclusive), terminated with
1,0.

@bug Different targets can use different lists
*/
range_t *expose_csr = NULL;

/** In addition to the ones in the standard spec, we'll also expose additional custom registers. */
range_t *expose_custom = NULL;

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

void
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

int
idcode_scan(struct jtag_tap *const tap,
	uint32_t p_in_value[1])
{
	assert(p_in_value);
	return uint32_instruction_scan(tap, DTM_IDCODE, "IDCODE", 0, p_in_value);
}

int
dtmcontrol_scan(struct jtag_tap *const tap,
	uint32_t const out_value,
	uint32_t *const p_in_value)
{
	return uint32_instruction_scan(tap, DTM_DTMCS, "DTMCONTROL", out_value, p_in_value);
}

/**

@todo Replace target->type by examine
*/
static struct target_type const *
__attribute__((warn_unused_result, pure))
get_target_type(struct target *const target)
{
	assert(target);
	struct riscv_info_t const *const info = target->arch_info;

	if (!info) {
		LOG_ERROR("%s: Target has not been initialized", target_name(target));
		return NULL;
	}

	switch (info->dtm_version) {
		case 1:
			return &riscv_013_target;

		default:
			LOG_ERROR("%s: Unsupported DTM version: %d", target_name(target), info->dtm_version);
			return NULL;
	}
}

/** Create the shared RISC-V structure. 
	@see struct riscv_info_t
*/
static struct riscv_info_t *
__attribute__((warn_unused_result))
riscv_info_init(struct target *const target)
{
	struct riscv_info_t *const r = calloc(1, sizeof(struct riscv_info_t));

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

		for (size_t e = 0; e < RISCV_MAX_REGISTERS; ++e)
			r->harts[hart].registers[e].valid = false;
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

	struct riscv_info_t *const info = target->arch_info;
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
	struct target_type const *const tt = get_target_type(target);

	if (tt) {
		tt->deinit_target(target);
		struct riscv_info_t *const info = target->arch_info;
		free(info->reg_names);
		free(info);
	}

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

/** @return error code */
static int
oldriscv_halt(struct target *const target)
{
	struct target_type const *const tt = get_target_type(target);
	assert(tt && tt->halt);
	return tt->halt(target);
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
	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);
	tdata1 = set_field(tdata1, bpcontrol_r, trigger->read);
	tdata1 = set_field(tdata1, bpcontrol_w, trigger->write);
	tdata1 = set_field(tdata1, bpcontrol_x, trigger->execute);
	tdata1 = set_field(tdata1, bpcontrol_u, !!(rvi->harts[hartid].misa & (1 << ('U' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_s, !!(rvi->harts[hartid].misa & (1 << ('S' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_h, !!(rvi->harts[hartid].misa & (1 << ('H' - 'A'))));
	tdata1 |= bpcontrol_m;
	tdata1 = set_field(tdata1, bpcontrol_bpmatch, 0); /* exact match */
	tdata1 = set_field(tdata1, bpcontrol_bpaction, 0); /* cause bp exception */

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
	struct riscv_info_t const *const rvi = riscv_info(target);

	/* tselect is already set */
	if (0 != (tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD))) {
		/* Trigger is already in use, presumably by user code. */
		LOG_ERROR("%s: Trigger is already in use, presumably by user code", target_name(target));
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* address/data match trigger */
	tdata1 |= MCONTROL_DMODE(riscv_xlen(target));
	tdata1 = set_field(tdata1, MCONTROL_ACTION, MCONTROL_ACTION_DEBUG_MODE);
	tdata1 = set_field(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_EQUAL);
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

	struct riscv_info_t *const rvi = riscv_info(target);
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
			get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));

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
	struct riscv_info_t *const rvi = riscv_info(target);
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
oldriscv_step(struct target *const target,
	int const current,
	uint32_t const address,
	int const handle_breakpoints)
{
	struct target_type const *const tt = get_target_type(target);
	assert(tt && tt->step);
	return tt->step(target, current, address, handle_breakpoints);
}

/** @return error code */
static int
old_or_new_riscv_step(struct target *const target,
	int const current,
	target_addr_t const address,
	int const handle_breakpoints)
{
	LOG_DEBUG("%s: handle_breakpoints=%d",
		target_name(target), handle_breakpoints);

	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);

	if (!rvi->is_halted)
		return oldriscv_step(target, current, address, handle_breakpoints);
	else
		return riscv_openocd_step(target, current, address, handle_breakpoints);
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
	struct riscv_info_t *const info = target->arch_info;
	info->dtm_version = get_field(dtmcontrol, DTMCONTROL_VERSION);
	LOG_DEBUG("%s:  version=0x%x", target_name(target), info->dtm_version);

	struct target_type const *const tt = get_target_type(target);

	if (!tt)
		return ERROR_TARGET_INVALID;

	{
		int const error_code = tt->init_target(info->cmd_ctx, target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	assert(tt->examine);
	return tt->examine(target);
}

/** @return error code */
static int
oldriscv_poll(struct target *const target)
{
	struct target_type const *const tt = get_target_type(target);
	assert(tt && tt->poll);
	return tt->poll(target);
}

/**
@bug dynamic dispatch
@return error code
*/
static int
old_or_new_riscv_poll(struct target *const target)
{
	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);
	return rvi->is_halted ? riscv_openocd_poll(target) : oldriscv_poll(target);
}

/**
@return error code
@bug dynamic dispatch
*/
static int
old_or_new_riscv_halt(struct target *const target)
{
	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);
	return rvi->is_halted ? riscv_openocd_halt(target) : oldriscv_halt(target);
}

/** @return error code */
static int
riscv_assert_reset(struct target *const target)
{
	struct target_type const *const tt = get_target_type(target);
	return tt->assert_reset(target);
}

/** @return error code */
static int
riscv_deassert_reset(struct target *const target)
{
	assert(target);
	LOG_DEBUG("%s: RISCV DEASSERT RESET", target_name(target));
	struct target_type const *const tt = get_target_type(target);
	return tt->deassert_reset(target);
}

/** @return error code */
static int
oldriscv_resume(struct target *const target,
	int const current,
	uint32_t const address,
	int const handle_breakpoints,
	int const debug_execution)
{
	struct target_type const *const tt = get_target_type(target);
	assert(tt && tt->resume);
	return
		tt->resume(target, current, address, handle_breakpoints, debug_execution);
}

/** @return error code */
static int
old_or_new_riscv_resume(struct target *const target,
	int const current,
	target_addr_t const address,
	int const handle_breakpoints,
	int debug_execution)
{
	assert(target);
	LOG_DEBUG("%s: handle_breakpoints=%d",
			target_name(target),
			handle_breakpoints);

	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);

	return
		rvi->is_halted ?
		riscv_openocd_resume(target, current, address, handle_breakpoints, debug_execution) :
		oldriscv_resume(target, current, address, handle_breakpoints, debug_execution);
}

/** @return error code */
static int
riscv_select_current_hart(struct target *const target)
{
	if (riscv_rtos_enabled(target)) {
		struct riscv_info_t *const rvi = riscv_info(target);
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

	struct target_type const *const tt = get_target_type(target);
	assert(tt);
	return tt->read_memory(target, address, size, count, buffer);
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

	struct target_type const *const tt = get_target_type(target);
	assert(tt);
	return tt->write_memory(target, address, size, count, buffer);
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

	struct riscv_info_t const *const rvi = riscv_info(target);
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

/** @return error code */
static int
riscv_arch_state(struct target *const target)
{
	struct target_type const *const tt = get_target_type(target);
	assert(tt);
	return tt->arch_state(target);
}

#if 0
/** Algorithm must end with a software breakpoint instruction. */
static int
riscv_run_algorithm(struct target *const target,
	int const num_mem_params,
	struct mem_param *const mem_params,
	int const num_reg_params,
	struct reg_param *const reg_params,
	target_addr_t const entry_point,
	target_addr_t const exit_point,
	int const timeout_ms,
	void *const arch_info)
{
	assert(target);
	struct riscv_info_t *const info = target->arch_info;

	if (0 < num_mem_params) {
		LOG_ERROR("%s: Memory parameters are not supported for RISC-V algorithms.",
				target_name(target));
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (TARGET_HALTED != target->state) {
		LOG_WARNING("%s: target not halted", target_name(target));
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Save registers */
	struct reg *const reg_pc =
		register_get_by_name(target->reg_cache, "pc", 1);

	if (!reg_pc)
		return ERROR_TARGET_INVALID;

	{
		int const error_code = reg_pc->type->get(reg_pc);

		if (ERROR_OK != error_code)
			return error_code;
	}

	uint64_t const saved_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	uint64_t saved_regs[32];

	for (int i = 0; i < num_reg_params; ++i) {
		LOG_DEBUG("%s: save %s", target_name(target), reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);

		if (!r) {
			LOG_ERROR("%s: Couldn't find register named '%s'",
					target_name(target),
					reg_params[i].reg_name);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		if (r->size != reg_params[i].size) {
			LOG_ERROR("%s: Register %s is %d bits instead of %d bits.",
					target_name(target),
					reg_params[i].reg_name,
					r->size,
					reg_params[i].size);
			return ERROR_TARGET_INVALID;
		}

		if (GDB_REGNO_XPR31 < r->number) {
			LOG_ERROR("%s: Only GPRs can be use as argument registers.",
					target_name(target));
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}

		{
			int const error_code = r->type->get(r);

			if (ERROR_OK != error_code)
				return error_code;
		}

		saved_regs[r->number] = buf_get_u64(r->value, 0, r->size);

		{
			int const error_code = r->type->set(r, reg_params[i].value);

			if (ERROR_OK != error_code)
				return error_code;
		}
	}


	/* Disable Interrupts before attempting to run the algorithm. */
	LOG_DEBUG("%s: Disabling Interrupts",
			target_name(target));

	struct reg *reg_mstatus =
		register_get_by_name(target->reg_cache, "mstatus", 1);

	if (!reg_mstatus) {
		LOG_ERROR("%s: Couldn't find mstatus!",
				target_name(target));
		return ERROR_TARGET_INVALID;
	}

	assert(reg_mstatus->type && reg_mstatus->type->get);
	reg_mstatus->type->get(reg_mstatus);
	uint64_t const current_mstatus = buf_get_u64(reg_mstatus->value, 0, reg_mstatus->size);
	{
		uint64_t const ie_mask = MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE;
		uint8_t mstatus_bytes[8];
		/**
		@todo check hart number
		*/
		buf_set_u64(mstatus_bytes, 0, info->harts[0].xlen, set_field(current_mstatus, ie_mask, 0));
		assert(reg_mstatus->type->set);
		reg_mstatus->type->set(reg_mstatus, mstatus_bytes);
	}

	/* Run algorithm */
	LOG_DEBUG("%s: resume at 0x%" TARGET_PRIxADDR,
			target_name(target),
			entry_point);

	{
		int const error_code = oldriscv_resume(target, 0, entry_point, 0, 0);

		if (ERROR_OK != error_code)
			return error_code;
	}

	{
		int64_t const start = timeval_ms();

		while (target->state != TARGET_HALTED) {
			LOG_DEBUG("%s: poll()", target_name(target));
			int64_t const now = timeval_ms();

			if (now > start + timeout_ms) {
				LOG_ERROR("%s: Algorithm timed out after %d ms." "\n"
						"  now   = 0x%08" PRIx64 "\n"
						"  start = 0x%08" PRIx64,
						target_name(target),
						timeout_ms,
						now,
						start);

				/**
				@bug oldriscv_halt
				*/
				oldriscv_halt(target);
				old_or_new_riscv_poll(target);
				return ERROR_TARGET_TIMEOUT;
			}

			{
				int const error_code = old_or_new_riscv_poll(target);

				if (ERROR_OK != error_code)
					return error_code;
			}
		}
	}

	{
		int const error_code = reg_pc->type->get(reg_pc);

		if (ERROR_OK != error_code)
			return error_code;
	}

	uint64_t const final_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	if (final_pc != exit_point) {
		LOG_ERROR("%s: PC ended up at 0x%" PRIx64 " instead of 0x%"
				TARGET_PRIxADDR,
				target_name(target),
				final_pc, exit_point);
		return ERROR_TARGET_FAILURE;
	}

	{
		/* Restore Interrupts */
		/**
		@todo restore interrupts on error too
		*/
		LOG_DEBUG("%s: Restoring Interrupts", target_name(target));
		uint8_t mstatus_bytes[8];
		buf_set_u64(mstatus_bytes, 0, info->harts[0].xlen, current_mstatus);
		reg_mstatus->type->set(reg_mstatus, mstatus_bytes);
	}

	/* Restore registers */
	uint8_t buf[8];
	/**
	@todo check hart number
	*/
	buf_set_u64(buf, 0, info->harts[0].xlen, saved_pc);

	{
		int const error_code = reg_pc->type->set(reg_pc, buf);

		if (ERROR_OK != error_code)
			return error_code;
	}

	for (int i = 0; i < num_reg_params; ++i) {
		LOG_DEBUG("%s: restore %s", target_name(target), reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		assert(r && r->number < DIM(saved_regs));
		buf_set_u64(buf, 0, info->harts[0].xlen, saved_regs[r->number]);

		{
			assert(r->type && r->type->set);
			int const error_code = r->type->set(r, buf);

			if (ERROR_OK != error_code)
				return error_code;
		}
	}

	return ERROR_OK;
}

/** Should run code on the target to perform CRC of memory.

	@todo Not yet implemented.
*/
static int
riscv_checksum_memory(struct target *const target,
	target_addr_t const address,
	uint32_t const count,
	uint32_t *const checksum)
{
	assert(checksum);
	*checksum = 0xFFFFFFFF;
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}
#endif

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
		struct riscv_info_t const *const rvi = riscv_info(target);
		assert(rvi && rvi->on_halt);
		rvi->on_halt(target);
		return RPH_DISCOVERED_HALTED;
	} else if (target->state != TARGET_RUNNING && !halted) {
		LOG_DEBUG("%s:  triggered running", target_name(target));
		target->state = TARGET_RUNNING;
		return RPH_DISCOVERED_RUNNING;
	}

	return RPH_NO_CHANGE;
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

	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi && rvi->halt_current_hart);
	return rvi->halt_current_hart(target);
}

int
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
		if (riscv_semihosting(target, &retval) != 0)
			return retval;
	}

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return ERROR_OK;
}

int
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
		struct riscv_info_t const *const rvi = riscv_info(target);
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
	struct riscv_info_t const *const rvi = riscv_info(target);
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
	assert(rvi->on_step);
	rvi->on_step(target);

	{
		assert(rvi->step_current_hart);
		int const error_code = rvi->step_current_hart(target);

		if (ERROR_OK != error_code)
			return error_code;
	}

	riscv_invalidate_register_cache(target);
	{
		assert(rvi->on_halt);
		int const error_code = rvi->on_halt(target);

		if (!riscv_is_halted(target)) {
			LOG_ERROR("%s: Hart was not halted after single step!", target_name(target));
			return ERROR_OK != error_code ? error_code : ERROR_TARGET_NOT_HALTED;
		}
	}

	return ERROR_OK;
}

int
riscv_openocd_resume(struct target *const target,
	int const current,
	target_addr_t const address,
	int const handle_breakpoints,
	int const debug_execution)
{
	assert(target);
	LOG_DEBUG("%s: debug_reason=%d", target_name(target), target->debug_reason);

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

int
riscv_openocd_step(struct target *const target,
	int const current,
	target_addr_t const address,
	int const handle_breakpoints)
{
	LOG_DEBUG("%s: stepping rtos hart", target_name(target));

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

	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);

	if (rvi->test_compliance) {
		return rvi->test_compliance(target);
	} else {
		LOG_ERROR("%s: This target does not support this command"
				" (may implement an older version of the spec).",
				target_name(target));
		return ERROR_TARGET_INVALID;
	}
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

	struct riscv_info_t const *const rvi = riscv_info(target);

	if (!rvi) {
		LOG_ERROR("%s: riscv_info is NULL!",
			target_name(target));
		return ERROR_TARGET_INVALID;
	}

	if (rvi->authdata_read) {
		uint32_t value;

		{
			int const error_code = rvi->authdata_read(target, &value);

			if (ERROR_OK != error_code)
				return error_code;
		}

		command_print(CMD_CTX, "0x%" PRIx32, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("%s: authdata_read is not implemented for this target.", target_name(target));
		return ERROR_TARGET_INVALID;
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

	struct riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);

	if (rvi->authdata_write) {
		return rvi->authdata_write(target, value);
	} else {
		LOG_ERROR("%s: authdata_write is not implemented for this target.", target_name(target));
		return ERROR_TARGET_INVALID;
	}
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

	struct riscv_info_t const *const rvi = riscv_info(target);

	if (!rvi) {
		LOG_ERROR("%s: riscv_info is NULL!", target_name(target));
		return ERROR_TARGET_INVALID;
	}

	if (rvi->dmi_read) {
		uint32_t address;
		uint32_t value;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

		{
			int const error_code = rvi->dmi_read(target, &value, address);

			if (ERROR_OK != error_code)
				return error_code;
		}

		command_print(CMD_CTX, "0x%" PRIx32, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("%s: dmi_read is not implemented for this target.", target_name(target));
		return ERROR_TARGET_INVALID;
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
	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);

	if (rvi->dmi_write) {
		return rvi->dmi_write(target, address, value);
	} else {
		LOG_ERROR("%s: dmi_write is not implemented for this target.", target_name(target));
		return ERROR_TARGET_INVALID;
	}
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
	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi);

	if (rvi->test_sba_config_reg) {
		return
			rvi->test_sba_config_reg(target, legal_address, num_words, illegal_address, run_sbbusyerror_test);
	} else {
		LOG_ERROR("%s: test_sba_config_reg is not implemented for this target.", target_name(target));
		return ERROR_TARGET_INVALID;
	}
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

/** @return error code*/
int
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

	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi && rvi->on_resume);
	{
		int const error_code = rvi->on_resume(target);

		if (ERROR_OK != error_code)
			return error_code;
	}
	assert(rvi->resume_current_hart);
	return rvi->resume_current_hart(target);
}

/** @return error code*/
int
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

/** @return error code */
int
riscv_set_current_hartid(struct target *const target,
	int const hartid)
{
	struct riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);

	if (!rvi->select_current_hart)
		return ERROR_OK;

	int const previous_hartid = riscv_current_hartid(target);
	rvi->current_hartid = hartid;
	assert(riscv_hart_enabled(target, hartid));
	LOG_DEBUG("%s: setting hartid to %d, was %d", target_name(target), hartid, previous_hartid);

	{
		int const error_code = rvi->select_current_hart(target);

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

void
riscv_invalidate_register_cache(struct target *const target)
{
	register_cache_invalidate(target->reg_cache);

	for (size_t i = 0; i < target->reg_cache->num_regs; ++i) {
		struct reg *reg = &target->reg_cache->reg_list[i];
		assert(reg);
		reg->valid = false;
	}

	struct riscv_info_t *const rvi = riscv_info(target);
	assert(rvi);
	rvi->registers_initialized = true;
}

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

	struct riscv_info_t const *const rvi = riscv_info(target);

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
	enum gdb_regno const gdb_reg_no,
	riscv_reg_t const value)
{
	return riscv_set_register_on_hart(target, riscv_current_hartid(target), gdb_reg_no, value);
}

/**	@return error code */
int
riscv_set_register_on_hart(struct target *const target,
	int const hartid,
	enum gdb_regno const regid,
	uint64_t const value)
{
	LOG_DEBUG("%s: [%d] %s <- %" PRIx64, target_name(target), hartid, gdb_regno_name(regid), value);
	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi && rvi->set_register);
	return rvi->set_register(target, hartid, regid, value);
}

/**	@note Syntactical sugar
	@return error code
*/
int
riscv_get_register(struct target *const target,
	riscv_reg_t *const value,
	enum gdb_regno const r)
{
	return
		riscv_get_register_on_hart(target, value, riscv_current_hartid(target), r);
}

/** @return error code*/
int
riscv_get_register_on_hart(struct target *const target,
	riscv_reg_t *const value/**<[out]*/,
	int const hartid,
	enum gdb_regno const regid)
{
	struct riscv_info_t const *const rvi = riscv_info(target);

	if (riscv_current_hartid(target) != hartid)
		riscv_invalidate_register_cache(target);

	int const err = rvi->get_register(target, value, hartid, regid);

	if (riscv_current_hartid(target) != hartid)
		riscv_invalidate_register_cache(target);

	assert(value);
	LOG_DEBUG("%s: [%d] %s: %" PRIx64, target_name(target), hartid, gdb_regno_name(regid), *value);
	return err;
}

bool
riscv_is_halted(struct target *const target)
{
	struct riscv_info_t const *const rvi = riscv_info(target);
	assert(rvi && rvi->is_halted);
	return rvi->is_halted(target);
}

enum riscv_halt_reason
	riscv_halt_reason(struct target *const target,
		int const hartid)
{
	struct riscv_info_t const *const rvi = riscv_info(target);

	if (ERROR_OK != riscv_set_current_hartid(target, hartid))
		return RISCV_HALT_ERROR;

	if (!riscv_is_halted(target)) {
		LOG_ERROR("%s: Hart is not halted!", target_name(target));
		return RISCV_HALT_UNKNOWN;
	}

	return rvi->halt_reason(target);
}

bool
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

/**	@brief Count triggers, and initialize trigger_count for each hart.

	trigger_count is initialized even if this function fails to discover something.

	Disable any hardware triggers that have @c dmode set.
	We can't have set them ourselves.
	Maybe they're left over from some killed debug session.
*/
/** @return error code*/
int
riscv_enumerate_triggers(struct target *const target)
{
	struct riscv_info_t *const rvi = riscv_info(target);
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
			tselect_rb &= ~(1ULL << (riscv_xlen(target)-1));

			if (tselect_rb != t)
				break;

			uint64_t tdata1;
			{
				int const error_code =
					riscv_get_register_on_hart(target, &tdata1, hartid, GDB_REGNO_TDATA1);

				if (ERROR_OK != error_code)
					return error_code;
			}

			int const type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));

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

char const *
gdb_regno_name(enum gdb_regno const regno)
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
				static char buf[32] = {[31]='\0'};

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

struct csr_info {
	unsigned number;
	char const *name;
};

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

static struct csr_info csr_info[] = {
#define DECLARE_CSR(name, number) { number, #name },
#include "encoding.h"
#undef DECLARE_CSR
};

/** @return error code*/
int
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
	struct riscv_info_t *const rvi = riscv_info(target);
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

struct target_type riscv_target = {
	.name = "riscv",
	.poll = old_or_new_riscv_poll,
	.arch_state = riscv_arch_state,
#if 0
	.target_request_data = NULL,
#endif
	.halt = old_or_new_riscv_halt,
	.resume = old_or_new_riscv_resume,
	.step = old_or_new_riscv_step,
	.assert_reset = riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,
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
