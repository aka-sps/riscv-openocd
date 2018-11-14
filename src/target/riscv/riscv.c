﻿#include "riscv.h"
#include "opcodes.h"

#include "target/algorithm.h"
#include "target/target_type.h"
#include "jtag/jtag.h"
#include "target/register.h"
#include "target/breakpoints.h"
#include "helper/time_support.h"
#include "rtos/rtos.h"

/*
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
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

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

/** JTAG registers. */
/** @{ */
#define DTMCONTROL					0x10
#define DTMCONTROL_VERSION			(0xf)
#define DBUS						0x11
/** @} */

struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read, write, execute;
	int unique_id;
};

struct range_s {
	uint16_t low;
	uint16_t high;
};
typedef struct range_s range_t;

static uint8_t const ir_dtmcontrol[1] = {DTMCONTROL};
static uint8_t const ir_dbus[1] = {DBUS};
static uint8_t const ir_idcode[1] = {0x1};
struct scan_field select_dtmcontrol = {
	.in_value = NULL,
	.out_value = ir_dtmcontrol
};

struct scan_field select_dbus = {
	.in_value = NULL,
	.out_value = ir_dbus
};

struct scan_field select_idcode = {
	.in_value = NULL,
	.out_value = ir_idcode
};


/** Wall-clock timeout for a command/access. Settable via RISC-V Target commands.*/
int riscv_command_timeout_sec = DEFAULT_COMMAND_TIMEOUT_SEC;

/** Wall-clock timeout after reset. Settable via RISC-V Target commands.*/
int riscv_reset_timeout_sec = DEFAULT_RESET_TIMEOUT_SEC;

/** @bug uninitialized */
bool riscv_prefer_sba;

/** In addition to the ones in the standard spec, we'll also expose additional
 * CSRs in this list.
 * The list is either NULL, or a series of ranges (inclusive), terminated with
 * 1,0. */
range_t *expose_csr;

/** Same, but for custom registers. */
range_t *expose_custom;

static uint32_t
dtmcontrol_scan(struct target *const target,
	uint32_t const out)
{
	assert(target);
	jtag_add_ir_scan(target->tap, &select_dtmcontrol, TAP_IDLE);

	uint8_t out_value[4];
	buf_set_u32(out_value, 0, 32, out);
	uint8_t in_value[4];
	struct scan_field const field = {
		.num_bits = 32,
		.out_value = out_value,
		.in_value = in_value,
	};
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	/* Always return to dbus. */
	/** @bug Non robust strategy */
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	{
		int const err = jtag_execute_queue();

		if (ERROR_OK != err) {
			LOG_ERROR("%s: failed jtag scan: %d",
				target->cmd_name, err);
			/** @todo Propagate error code */
			/** @bug Invalid result on error */
			return 0xBADC0DE;
		}
	}

	uint32_t const in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("%s: DTMCONTROL: 0x%" PRIx32 " -> 0x%" PRIx32,
		target->cmd_name, out, in);
	return in;
}

static struct target_type const *
__attribute__((warn_unused_result))
get_target_type(struct target *const target)
{
	assert(target);
	struct riscv_info_t const *const info = target->arch_info;

	if (!info) {
		LOG_ERROR("%s: Target has not been initialized", target->cmd_name);
		return NULL;
	}

	switch (info->dtm_version) {
		case 0:
			return &riscv011_target;

		case 1:
			return &riscv013_target;

		default:
			LOG_ERROR("%s: Unsupported DTM version: %d", target->cmd_name, info->dtm_version);
			return NULL;
	}
}

/** Initializes the shared RISC-V structure. */
static struct riscv_info_t *
__attribute__((warn_unused_result))
riscv_info_init(struct target *const target)
{
	struct riscv_info_t *const r = calloc(1, sizeof(struct riscv_info_t));

	if (!r) {
		LOG_ERROR("%s: Fatal: No free memory!", target->cmd_name);
		return NULL;
	}

	memset(r, 0, sizeof(*r));
	r->dtm_version = 1;
	r->registers_initialized = false;
	r->current_hartid = target->coreid;

	memset(r->trigger_unique_id, 0xff, sizeof(r->trigger_unique_id));

	for (size_t hart = 0; hart < RISCV_MAX_HARTS; ++hart) {
		r->harts[hart].xlen = -1;

		for (size_t e = 0; e < RISCV_MAX_REGISTERS; ++e)
			r->harts[hart].registers[e].valid = false;
	}

	return r;
}

static int
riscv_init_target(struct command_context *const cmd_ctx,
	struct target *const target)
{
	assert(target);
	LOG_DEBUG("%s: riscv_init_target()", target->cmd_name);
	target->arch_info = riscv_info_init(target);

	if (!target->arch_info)
		return ERROR_TARGET_INVALID;

	struct riscv_info_t *const info = target->arch_info;
	info->cmd_ctx = cmd_ctx;

	assert(target->tap);
	select_dtmcontrol.num_bits = target->tap->ir_length;
	select_dbus.num_bits = target->tap->ir_length;
	select_idcode.num_bits = target->tap->ir_length;

	riscv_semihosting_init(target);

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static void
riscv_deinit_target(struct target *const target)
{
	LOG_DEBUG("%s: riscv_deinit_target()", target->cmd_name);
	struct target_type const *const tt = get_target_type(target);

	if (tt) {
		tt->deinit_target(target);
		struct riscv_info_t *const info = target->arch_info;
		free(info->reg_names);
		free(info);
	}

	/* Free the shared structure use for most registers. */
	free(target->reg_cache->reg_list[0].arch_info);

	/* Free the ones we allocated separately. */
	for (unsigned i = GDB_REGNO_COUNT; i < target->reg_cache->num_regs; ++i)
		free(target->reg_cache->reg_list[i].arch_info);

	free(target->reg_cache->reg_list);
	free(target->reg_cache);
	target->arch_info = NULL;
}

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
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	assert(trigger);
	struct riscv_info_t *const r = riscv_info(target);
	assert(r);
	tdata1 = set_field(tdata1, bpcontrol_r, trigger->read);
	tdata1 = set_field(tdata1, bpcontrol_w, trigger->write);
	tdata1 = set_field(tdata1, bpcontrol_x, trigger->execute);
	tdata1 = set_field(tdata1, bpcontrol_u, !!(r->harts[hartid].misa & (1 << ('U' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_s, !!(r->harts[hartid].misa & (1 << ('S' - 'A'))));
	tdata1 = set_field(tdata1, bpcontrol_h, !!(r->harts[hartid].misa & (1 << ('H' - 'A'))));
	tdata1 |= bpcontrol_m;
	tdata1 = set_field(tdata1, bpcontrol_bpmatch, 0); /* exact match */
	tdata1 = set_field(tdata1, bpcontrol_bpaction, 0); /* cause bp exception */

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);

	riscv_reg_t tdata1_rb;
	{
		int const ret = riscv_get_register_on_hart(target, &tdata1_rb, hartid, GDB_REGNO_TDATA1);

		if (ERROR_OK != ret)
			return ret;
	}
	LOG_DEBUG("%s: tdata1=0x%" PRIx64, target->cmd_name, tdata1_rb);

	if (tdata1 != tdata1_rb) {
		LOG_DEBUG("%s: Trigger doesn't support what we need; After writing 0x%"
				PRIx64 " to tdata1 it contains 0x%" PRIx64,
				target->cmd_name,
				tdata1,
				tdata1_rb);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA2, trigger->address);

	return ERROR_OK;
}

static int
maybe_add_trigger_t2(struct target *const target,
	unsigned const hartid,
	struct trigger *const trigger,
	uint64_t tdata1)
{
	struct riscv_info_t *const r = riscv_info(target);

	/* tselect is already set */
	if (0 != (tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD))) {
		/* Trigger is already in use, presumably by user code. */
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* address/data match trigger */
	tdata1 |= MCONTROL_DMODE(riscv_xlen(target));
	tdata1 = set_field(tdata1, MCONTROL_ACTION, MCONTROL_ACTION_DEBUG_MODE);
	tdata1 = set_field(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_EQUAL);
	tdata1 |= MCONTROL_M;

	if (r->harts[hartid].misa & (1 << ('H' - 'A')))
		tdata1 |= MCONTROL_H;

	if (r->harts[hartid].misa & (1 << ('S' - 'A')))
		tdata1 |= MCONTROL_S;

	if (r->harts[hartid].misa & (1 << ('U' - 'A')))
		tdata1 |= MCONTROL_U;

	assert(trigger);
	if (trigger->execute)
		tdata1 |= MCONTROL_EXECUTE;

	if (trigger->read)
		tdata1 |= MCONTROL_LOAD;

	if (trigger->write)
		tdata1 |= MCONTROL_STORE;

	{
		int const err = riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, tdata1);
		if (ERROR_OK != err)
			return err;
	}

	uint64_t tdata1_rb;
	{
		int const err =
			riscv_get_register_on_hart(target, &tdata1_rb, hartid, GDB_REGNO_TDATA1);
		if (ERROR_OK != err)
			return err;
	}

	LOG_DEBUG("%s: tdata1=0x%" PRIx64, target->cmd_name, tdata1_rb);

	if (tdata1 != tdata1_rb) {
		LOG_DEBUG("%s: Trigger doesn't support what we need; After writing 0x%"
				PRIx64 " to tdata1 it contains 0x%" PRIx64,
				target->cmd_name,
				tdata1, tdata1_rb);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA2, trigger->address);
}

static int
add_trigger(struct target *const target,
	struct trigger *const trigger)
{
	struct riscv_info_t *const r = riscv_info(target);

	{
		int const err = riscv_enumerate_triggers(target);
		if (ERROR_OK != err)
			return err;
	}

	/** @details In RTOS mode, we need to set the same trigger in the same slot on every hart,
	to keep up the illusion that each hart is a thread running on the same core. 
	
	Otherwise, we just set the trigger on the one hart this target deals with.
	*/

	riscv_reg_t tselect[RISCV_MAX_HARTS];

	int first_hart = -1;

	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		if (first_hart < 0)
			first_hart = hartid;

		{
			int const err =
				riscv_get_register_on_hart(target, &tselect[hartid], hartid, GDB_REGNO_TSELECT);

			if (ERROR_OK != err)
				return err;
		}
	}

	assert(first_hart >= 0);

	unsigned i;

	for (i = 0; i < r->harts[first_hart].trigger_count; ++i) {
		if (r->trigger_unique_id[i] != -1)
			continue;

		riscv_set_register_on_hart(target, first_hart, GDB_REGNO_TSELECT, i);

		uint64_t tdata1;
		{
			int const err =
				riscv_get_register_on_hart(target, &tdata1, first_hart, GDB_REGNO_TDATA1);

			if (ERROR_OK != err)
				return err;
		}

		int const type =
			get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));

		{
			int err = ERROR_OK;

			for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
				if (!riscv_hart_enabled(target, hartid))
					continue;

				if (hartid > first_hart)
					riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, i);

				switch (type) {
				case 1:
					err = maybe_add_trigger_t1(target, hartid, trigger, tdata1);
					break;

				case 2:
					err = maybe_add_trigger_t2(target, hartid, trigger, tdata1);
					break;

				default:
					LOG_DEBUG("%s: trigger %d has unknown type %d", target->cmd_name, i, type);
					continue;
				}

				if (err != ERROR_OK)
					continue;
			}

			if (ERROR_OK != err)
				continue;
		}

		assert(trigger);
		LOG_DEBUG("%s: Using trigger %d (type %d) for bp %d",
				target->cmd_name,
				i,
				type,
				trigger->unique_id);
		r->trigger_unique_id[i] = trigger->unique_id;
		break;
	}

	for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		int const err =
			riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect[hartid]);
		if (ERROR_OK != err)
			return err;
	}

	if (i >= r->harts[first_hart].trigger_count) {
		LOG_ERROR("%s: Couldn't find an available hardware trigger.", target->cmd_name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return ERROR_OK;
}

static int
riscv_add_breakpoint(struct target *const target,
	struct breakpoint *const breakpoint)
{
	assert(breakpoint);

	switch (breakpoint->type) {
	case BKPT_SOFT:
		{
			/** @todo check RVC for size/alignment */
			if (!(breakpoint->length == 4 || breakpoint->length == 2)) {
				LOG_ERROR("%s: Invalid breakpoint length %d",
					target->cmd_name,
					breakpoint->length);
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}

			if (0 != (breakpoint->address % 2)) {
				LOG_ERROR("%s: Invalid breakpoint alignment for address 0x%" TARGET_PRIxADDR,
					target->cmd_name,
					breakpoint->address);
				return ERROR_TARGET_UNALIGNED_ACCESS;
			}

			{
				int const err =
					target_read_memory(target, breakpoint->address, 2, breakpoint->length / 2, breakpoint->orig_instr);

				if (ERROR_OK != err) {
					LOG_ERROR("%s: Failed to read original instruction at 0x%" TARGET_PRIxADDR,
						target->cmd_name,
						breakpoint->address);
					return err;
				}
			}

			uint8_t buff[4];
			buf_set_u32(buff, 0, breakpoint->length * CHAR_BIT, breakpoint->length == 4 ? ebreak() : ebreak_c());

			{
				int const err =
					target_write_memory(target, breakpoint->address, 2, breakpoint->length / 2, buff);

				if (ERROR_OK != err) {
					LOG_ERROR("%s: Failed to write %d-byte breakpoint instruction at 0x%" TARGET_PRIxADDR,
						target->cmd_name, breakpoint->length, breakpoint->address);
					return err;
				}
			}
		}
		break;

	case BKPT_HARD:
		{
			struct trigger trigger;
			trigger_from_breakpoint(&trigger, breakpoint);
			{
				int const err =
					add_trigger(target, &trigger);

				if (ERROR_OK != err)
					return err;
			}
		}
		break;

	default:
		LOG_INFO("%s: OpenOCD only supports hardware and software breakpoints.",
			target->cmd_name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = true;
	return ERROR_OK;
}

static int
remove_trigger(struct target *const target,
	struct trigger *const trigger)
{
	struct riscv_info_t *const r = riscv_info(target);

	{
		int const err = riscv_enumerate_triggers(target);

		if (ERROR_OK != err)
			return err;
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
	unsigned i;

	for (i = 0; i < r->harts[first_hart].trigger_count; ++i) {
		if (r->trigger_unique_id[i] == trigger->unique_id)
			break;
	}

	if (i >= r->harts[first_hart].trigger_count) {
		LOG_ERROR("%s: Couldn't find the hardware resources used by hardware trigger.",
			target->cmd_name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	LOG_DEBUG("%s: Stop using resource %d for bp %d", target->cmd_name, i, trigger->unique_id);

	for (int hartid = first_hart; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		riscv_reg_t tselect;
		{
			int const err =
				riscv_get_register_on_hart(target, &tselect, hartid, GDB_REGNO_TSELECT);
			if (ERROR_OK != err)
				return err;
		}

		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, i);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);
		riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect);
	}

	r->trigger_unique_id[i] = -1;

	return ERROR_OK;
}

static int
riscv_remove_breakpoint(struct target *const target,
	struct breakpoint *breakpoint)
{
	assert(breakpoint);

	switch (breakpoint->type) {
	case BKPT_SOFT:
		{
			int const err =
				target_write_memory(target, breakpoint->address, 2, breakpoint->length / 2, breakpoint->orig_instr);

			if (ERROR_OK != err) {
				LOG_ERROR("%s: Failed to restore instruction for %d-byte breakpoint at "
					"0x%" TARGET_PRIxADDR,
					target->cmd_name,
					breakpoint->length,
					breakpoint->address);
				return err;
			}
		}
		break;

	case BKPT_HARD:
		{
			struct trigger trigger;
			trigger_from_breakpoint(&trigger, breakpoint);
			int const err = remove_trigger(target, &trigger);

			if (ERROR_OK != err)
				return err;

		}
		break;

	default:
		LOG_INFO("%s: OpenOCD only supports hardware and software breakpoints.",
			target->cmd_name);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = false;

	return ERROR_OK;
}

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

static int
riscv_add_watchpoint(struct target *const target,
	struct watchpoint *const watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	{
		int const err = add_trigger(target, &trigger);

		if (ERROR_OK != err)
			return err;
	}

	assert(watchpoint);
	watchpoint->set = true;

	return ERROR_OK;
}

static int
riscv_remove_watchpoint(struct target *const target,
	struct watchpoint *const watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	{
		int const err = remove_trigger(target, &trigger);
		if (ERROR_OK != err)
			return err;
	}

	assert(watchpoint);
	watchpoint->set = false;

	return ERROR_OK;
}

/* Sets *hit_watchpoint to the first watchpoint identified as causing the
 * current halt.
 *
 * The GDB server uses this information to tell GDB what data address has
 * been hit, which enables GDB to print the hit variable along with its old
 * and new value. */
static int
riscv_hit_watchpoint(struct target *const target,
	struct watchpoint **const hit_watchpoint)
{
	assert(target);
	struct watchpoint *wp = target->watchpoints;

	LOG_DEBUG("%s: Current hartid = %d",
			target->cmd_name,
			riscv_current_hartid(target));

	/** @todo instead of disassembling the instruction that we think caused the
	 * trigger, check the hit bit of each watchpoint first. The hit bit is
	 * simpler and more reliable to check but as it is optional and relatively
	 * new, not all hardware will implement it  */
	riscv_reg_t dpc;
	riscv_get_register(target, &dpc, GDB_REGNO_DPC);
	const uint8_t length = 4;
	LOG_DEBUG("%s: dpc is 0x%" PRIx64,
			target->cmd_name, dpc);

	/* fetch the instruction at dpc */
	uint8_t buffer[length];

	{
		int const err =target_read_buffer(target, dpc, length, buffer);

		if (ERROR_OK != err) {
			LOG_ERROR("%s: Failed to read instruction at dpc 0x%" PRIx64,
					target->cmd_name,
					dpc);
			return err;
		}
	}

	uint32_t instruction = 0;

	for (int i = 0; i < length; ++i) {
		LOG_DEBUG("%s: Next byte is %x",
				target->cmd_name,
				buffer[i]);
		instruction += (buffer[i] << 8 * i);

	}
	LOG_DEBUG("%s: Full instruction is %x",
			target->cmd_name,
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
					target->cmd_name,
					instruction);
			imm = ((instruction & 0xf80) >> 7) | ((instruction & 0xfe000000) >> 20);
		} else {
			LOG_DEBUG("%s: %x is load instruction",
					target->cmd_name,
					instruction);
			imm = (instruction & 0xfff00000) >> 20;
		}
		/* sign extend 12-bit imm to 16-bits */
		if (imm & (1 << 11))
			imm |= 0xf000;
		mem_addr += imm;
		LOG_DEBUG("%s: memory address=0x%" PRIx64,
				target->cmd_name,
				mem_addr);
	} else {
		LOG_DEBUG("%s: %x is not a RV32I load or store",
				target->cmd_name,
				instruction);
		return ERROR_TARGET_INVALID;
	}

	for (; wp; wp = wp->next) {
		/** @todo support length/mask */
		if (wp->address == mem_addr) {
			assert(hit_watchpoint);
			*hit_watchpoint = wp;
			LOG_DEBUG("%s: Hit address=%" TARGET_PRIxADDR,
					target->cmd_name,
					wp->address);
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


static int oldriscv_step(struct target *const target, int const current, uint32_t const address,
		int const handle_breakpoints)
{
	struct target_type const *const tt = get_target_type(target);
	assert(tt);
	return tt->step(target, current, address, handle_breakpoints);
}

static int old_or_new_riscv_step(struct target *const target,
	int const current,
	target_addr_t const address,
	int const handle_breakpoints)
{
	struct riscv_info_t *const r = riscv_info(target);
	LOG_DEBUG("%s: handle_breakpoints=%d", target->cmd_name, handle_breakpoints);

	if (r->is_halted == NULL)
		return oldriscv_step(target, current, address, handle_breakpoints);
	else
		return riscv_openocd_step(target, current, address, handle_breakpoints);
}

static int riscv_examine(struct target *const target)
{
	assert(target);
	LOG_DEBUG("%s: riscv_examine()", target->cmd_name);

	if (target_was_examined(target)) {
		LOG_DEBUG("%s: Target was already examined.", target->cmd_name);
		return ERROR_OK;
	}

	/* Don't need to select dbus, since the first thing we do is read dtmcontrol. */
	uint32_t const dtmcontrol = dtmcontrol_scan(target, 0);
	LOG_DEBUG("%s: dtmcontrol=0x%x", target->cmd_name, dtmcontrol);
	struct riscv_info_t *const info = target->arch_info;
	info->dtm_version = get_field(dtmcontrol, DTMCONTROL_VERSION);
	LOG_DEBUG("%s:  version=0x%x", target->cmd_name, info->dtm_version);

	struct target_type const *const tt = get_target_type(target);

	if (!tt)
		return ERROR_TARGET_INVALID;

	{
		int const err = tt->init_target(info->cmd_ctx, target);

		if (err != ERROR_OK)
			return err;
	}

	return tt->examine(target);
}

static int
oldriscv_poll(struct target *const target)
{
	struct target_type const *const tt = get_target_type(target);
	return tt->poll(target);
}

static int
old_or_new_riscv_poll(struct target *const target)
{
	struct riscv_info_t *const r = riscv_info(target);
	assert(r);
	return r->is_halted ? riscv_openocd_poll(target) : oldriscv_poll(target);
}

static int
old_or_new_riscv_halt(struct target *const target)
{
	struct riscv_info_t *const r = riscv_info(target);
	assert(r);
	return r->is_halted ? riscv_openocd_halt(target) : oldriscv_halt(target);
}

static int
riscv_assert_reset(struct target *const target)
{
	struct target_type const *const tt = get_target_type(target);
	return tt->assert_reset(target);
}

static int
riscv_deassert_reset(struct target *const target)
{
	assert(target);
	LOG_DEBUG("%s: RISCV DEASSERT RESET", target->cmd_name);
	struct target_type const *const tt = get_target_type(target);
	return tt->deassert_reset(target);
}

static int
oldriscv_resume(struct target *const target,
	int const current,
	uint32_t const address,
	int const handle_breakpoints,
	int const debug_execution)
{
	struct target_type const *const tt = get_target_type(target);
	assert(tt);
	return
		tt->resume(target, current, address, handle_breakpoints, debug_execution);
}

static int
old_or_new_riscv_resume(struct target *const target,
	int const current,
	target_addr_t const address,
	int const handle_breakpoints,
	int debug_execution)
{
	assert(target);
	LOG_DEBUG("%s: handle_breakpoints=%d",
			target->cmd_name,
			handle_breakpoints);

	struct riscv_info_t *const r = riscv_info(target);
	assert(r);

	return
		r->is_halted ?
		riscv_openocd_resume(target, current, address, handle_breakpoints, debug_execution) :
		oldriscv_resume(target, current, address, handle_breakpoints, debug_execution);
}

static int
riscv_select_current_hart(struct target *const target)
{
	if (riscv_rtos_enabled(target)) {
		struct riscv_info_t *const r = riscv_info(target);
		assert(r);

		if (r->rtos_hartid == -1)
			r->rtos_hartid = target->rtos->current_threadid - 1;
		return riscv_set_current_hartid(target, r->rtos_hartid);
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
				target->cmd_name,
				address,
				size);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (0 == count)
		return ERROR_OK;

	assert(buffer);

	{
		int const err = riscv_select_current_hart(target);

		if (ERROR_OK != err)
			return err;
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
				target->cmd_name,
				address,
				size);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (0 == count)
		return ERROR_OK;

	{
		int const err = riscv_select_current_hart(target);

		if (ERROR_OK != err)
			return err;
	}

	struct target_type const *const tt = get_target_type(target);
	assert(tt);
	return tt->write_memory(target, address, size, count, buffer);
}

static int
riscv_get_gdb_reg_list(struct target *const target,
	struct reg **reg_list[],
	int *const reg_list_size,
	enum target_register_class const reg_class)
{
	assert(target);
	LOG_DEBUG("%s: reg_class=%d",
			target->cmd_name,
			reg_class);

	struct riscv_info_t *const r = riscv_info(target);
	assert(r);
	LOG_DEBUG("%s: rtos_hartid=%d current_hartid=%d",
			target->cmd_name,
			r->rtos_hartid,
			r->current_hartid);

	if (!target->reg_cache) {
		LOG_ERROR("%s: Target not initialized.",
				target->cmd_name);
		return ERROR_TARGET_INIT_FAILED;
	}

	{
		int const err = riscv_select_current_hart(target);

		if (ERROR_OK != err)
			return err;
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
				target->cmd_name,
				reg_class);
			return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	*reg_list = calloc(*reg_list_size, sizeof(struct reg *));
	if (!*reg_list) {
		LOG_ERROR("%s: Fatal: No free memory!", target->cmd_name);
		return ERROR_FAIL;
	}

	for (int i = 0; i < *reg_list_size; ++i) {
		assert(!target->reg_cache->reg_list[i].valid || target->reg_cache->reg_list[i].size > 0);
		(*reg_list)[i] = &target->reg_cache->reg_list[i];
	}

	return ERROR_OK;
}

static int
riscv_arch_state(struct target *const target)
{
	struct target_type const *const tt = get_target_type(target);
	assert(tt);
	return tt->arch_state(target);
}

/* Algorithm must end with a software breakpoint instruction. */
static int
riscv_run_algorithm(struct target *const target, int const num_mem_params,
		struct mem_param *const mem_params, int const num_reg_params,
		struct reg_param *const reg_params, target_addr_t const entry_point,
		target_addr_t const exit_point, int const timeout_ms, void *const arch_info)
{
	assert(target);
	struct riscv_info_t *const info = target->arch_info;

	if (num_mem_params > 0) {
		LOG_ERROR("%s: Memory parameters are not supported for RISC-V algorithms.",
				target->cmd_name);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (TARGET_HALTED != target->state) {
		LOG_WARNING("%s: target not halted", target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Save registers */
	struct reg *const reg_pc =
		register_get_by_name(target->reg_cache, "pc", 1);

	if (!reg_pc)
		return ERROR_TARGET_INVALID;

	{
		int const err = reg_pc->type->get(reg_pc);

		if (ERROR_OK != err)
			return err;
	}

	uint64_t const saved_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	uint64_t saved_regs[32];

	for (int i = 0; i < num_reg_params; ++i) {
		LOG_DEBUG("%s: save %s", target->cmd_name, reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);

		if (!r) {
			LOG_ERROR("%s: Couldn't find register named '%s'",
					target->cmd_name,
					reg_params[i].reg_name);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		if (r->size != reg_params[i].size) {
			LOG_ERROR("%s: Register %s is %d bits instead of %d bits.",
					target->cmd_name,
					reg_params[i].reg_name,
					r->size,
					reg_params[i].size);
			return ERROR_TARGET_INVALID;
		}

		if (r->number > GDB_REGNO_XPR31) {
			LOG_ERROR("%s: Only GPRs can be use as argument registers.",
					target->cmd_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}

		{
			int const err = r->type->get(r);

			if (ERROR_OK != err)
				return err;
		}

		saved_regs[r->number] = buf_get_u64(r->value, 0, r->size);

		{
			int const err = r->type->set(r, reg_params[i].value);

			if (ERROR_OK != err)
				return err;
		}
	}


	/* Disable Interrupts before attempting to run the algorithm. */
	LOG_DEBUG("%s: Disabling Interrupts",
			target->cmd_name);

	struct reg *reg_mstatus =
		register_get_by_name(target->reg_cache, "mstatus", 1);

	if (!reg_mstatus) {
		LOG_ERROR("%s: Couldn't find mstatus!",
				target->cmd_name);
		return ERROR_TARGET_INVALID;
	}

	assert(reg_mstatus->type && reg_mstatus->type->get);
	reg_mstatus->type->get(reg_mstatus);
	uint64_t const current_mstatus = buf_get_u64(reg_mstatus->value, 0, reg_mstatus->size);
	{
		uint64_t const ie_mask = MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE;
		uint8_t mstatus_bytes[8];
		/** @todo check hart number */
		buf_set_u64(mstatus_bytes, 0, info->harts[0].xlen, set_field(current_mstatus, ie_mask, 0));
		assert(reg_mstatus->type->set);
		reg_mstatus->type->set(reg_mstatus, mstatus_bytes);
	}

	/* Run algorithm */
	LOG_DEBUG("%s: resume at 0x%" TARGET_PRIxADDR,
			target->cmd_name,
			entry_point);

	{
		int const err = oldriscv_resume(target, 0, entry_point, 0, 0);

		if (ERROR_OK != err)
			return err;
	}

	{
		int64_t const start = timeval_ms();

		while (target->state != TARGET_HALTED) {
			LOG_DEBUG("%s: poll()", target->cmd_name);
			int64_t const now = timeval_ms();

			if (now > start + timeout_ms) {
				LOG_ERROR("%s: Algorithm timed out after %d ms." "\n"
						"  now   = 0x%08" PRIx64 "\n"
						"  start = 0x%08" PRIx64,
						target->cmd_name,
						timeout_ms,
						now,
						start);

				/** @bug oldriscv_halt */
				oldriscv_halt(target);
				old_or_new_riscv_poll(target);
				return ERROR_TARGET_TIMEOUT;
			}

			{
				int const err = old_or_new_riscv_poll(target);

				if (ERROR_OK != err)
					return err;
			}
		}
	}

	{
		int const err = reg_pc->type->get(reg_pc);

		if (ERROR_OK != err)
			return err;
	}
	uint64_t const final_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);

	if (final_pc != exit_point) {
		LOG_ERROR("%s: PC ended up at 0x%" PRIx64 " instead of 0x%"
				TARGET_PRIxADDR,
				target->cmd_name,
				final_pc, exit_point);
		return ERROR_TARGET_FAILURE;
	}

	{
		/* Restore Interrupts */
		/** @todo restore interrupts on error too */
		LOG_DEBUG("%s: Restoring Interrupts", target->cmd_name);
		uint8_t mstatus_bytes[8];
		buf_set_u64(mstatus_bytes, 0, info->harts[0].xlen, current_mstatus);
		reg_mstatus->type->set(reg_mstatus, mstatus_bytes);
	}

	/* Restore registers */
	uint8_t buf[8];
	/** @todo check hart number */
	buf_set_u64(buf, 0, info->harts[0].xlen, saved_pc);

	{
		int const err = reg_pc->type->set(reg_pc, buf);

		if (ERROR_OK != err)
			return err;
	}

	for (int i = 0; i < num_reg_params; ++i) {
		LOG_DEBUG("%s: restore %s", target->cmd_name, reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, 0);
		assert(r && r->number < DIM(saved_regs));
		buf_set_u64(buf, 0, info->harts[0].xlen, saved_regs[r->number]);

		{
			assert(r->type && r->type->set);
			int const err = r->type->set(r, buf);

			if (ERROR_OK != err)
				return err;
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

enum riscv_poll_hart_e {
	RPH_NO_CHANGE,
	RPH_DISCOVERED_HALTED,
	RPH_DISCOVERED_RUNNING,
	RPH_ERROR
};

static enum riscv_poll_hart_e
riscv_poll_hart(struct target *const target, int const hartid)
{
	struct riscv_info_t *const r = riscv_info(target);

	if (ERROR_OK != riscv_set_current_hartid(target, hartid))
		return RPH_ERROR;

	LOG_DEBUG("%s: polling hart %d, target->state=%d", target->cmd_name, hartid, target->state);

	/* If OpenOCD thinks we're running but this hart is halted then it's time
	 * to raise an event. */
	bool const halted = riscv_is_halted(target);

	if (target->state != TARGET_HALTED && halted) {
		LOG_DEBUG("%s:  triggered a halt", target->cmd_name);
		r->on_halt(target);
		return RPH_DISCOVERED_HALTED;
	} else if (target->state != TARGET_RUNNING && !halted) {
		LOG_DEBUG("%s:  triggered running", target->cmd_name);
		target->state = TARGET_RUNNING;
		return RPH_DISCOVERED_RUNNING;
	}

	return RPH_NO_CHANGE;
}

static int
riscv_halt_one_hart(struct target *const target, int const hartid)
{
	struct riscv_info_t *const r = riscv_info(target);
	LOG_DEBUG("%s: halting hart %d", target->cmd_name, hartid);

	{
		int const err = riscv_set_current_hartid(target, hartid);

		if (ERROR_OK != err)
			return err;
	}

	if (riscv_is_halted(target)) {
		LOG_DEBUG("%s:  hart %d requested halt, but was already halted",
			target->cmd_name,
			hartid);
		return ERROR_OK;
	}

	return r->halt_current_hart(target);
}

int
riscv_openocd_poll(struct target *const target)
{
	assert(target);
	LOG_DEBUG("%s: polling all harts", target->cmd_name);
	int halted_hart = -1;

	if (riscv_rtos_enabled(target)) {
		/* Check every hart for an event. */
		for (int i = 0; i < riscv_count_harts(target); ++i) {
			enum riscv_poll_hart_e const out =
				riscv_poll_hart(target, i);

			switch (out) {
			case RPH_NO_CHANGE:
			case RPH_DISCOVERED_RUNNING:
				continue;

			case RPH_DISCOVERED_HALTED:
				halted_hart = i;
				break;

			case RPH_ERROR:
				return ERROR_TARGET_FAILURE;
			}
		}

		if (halted_hart == -1) {
			LOG_DEBUG("%s:  no harts just halted, target->state=%d", target->cmd_name, target->state);
			return ERROR_OK;
		}

		LOG_DEBUG("%s:  hart %d halted", target->cmd_name, halted_hart);

		/* If we're here then at least one hart triggered.  That means
		 * we want to go and halt _every_ hart in the system, as that's
		 * the invariant we hold here.	Some harts might have already
		 * halted (as we're either in single-step mode or they also
		 * triggered a breakpoint), so don't attempt to halt those
		 * harts. */
		for (int i = 0; i < riscv_count_harts(target); ++i)
			riscv_halt_one_hart(target, i);

	} else {
		enum riscv_poll_hart_e const out =
			riscv_poll_hart(target, riscv_current_hartid(target));

		if (out == RPH_NO_CHANGE || out == RPH_DISCOVERED_RUNNING)
			return ERROR_OK;
		else if (out == RPH_ERROR)
			return ERROR_TARGET_FAILURE;

		halted_hart = riscv_current_hartid(target);
		LOG_DEBUG("%s:  hart %d halted", target->cmd_name, halted_hart);
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
		return ERROR_TARGET_FAILURE;

	/** @bug no default case */
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
	LOG_DEBUG("%s: halting all harts", target->cmd_name);

	{
		int const err = riscv_halt_all_harts(target);

		if (ERROR_OK != err) {
			LOG_ERROR("%s: Unable to halt all harts", target->cmd_name);
			return err;
		}
	}

	register_cache_invalidate(target->reg_cache);

	if (riscv_rtos_enabled(target)) {
		struct riscv_info_t *const r =
			riscv_info(target);
		assert(r);

		if (r->rtos_hartid != -1) {
			LOG_DEBUG("%s: halt requested on RTOS hartid %d",
				target->cmd_name,
				r->rtos_hartid);
			target->rtos->current_threadid = r->rtos_hartid + 1;
			target->rtos->current_thread = r->rtos_hartid + 1;
		} else
			LOG_DEBUG("%s: halt requested, but no known RTOS hartid",
				target->cmd_name);
	}

	target->state = TARGET_HALTED;
	target->debug_reason = DBG_REASON_DBGRQ;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	return ERROR_OK;
}

int
riscv_step_rtos_hart(struct target *const target)
{
	struct riscv_info_t *const r = riscv_info(target);
	int hartid = r->current_hartid;

	if (riscv_rtos_enabled(target)) {
		hartid = r->rtos_hartid;
		if (hartid == -1) {
			LOG_DEBUG("%s: GDB has asked me to step \"any\" thread, so I'm stepping hart 0.", target->cmd_name);
			hartid = 0;
		}
	}

	{
		int const err = riscv_set_current_hartid(target, hartid);
		if (ERROR_OK != err)
			return err;
	}

	LOG_DEBUG("%s: stepping hart %d", target->cmd_name, hartid);

	if (!riscv_is_halted(target)) {
		LOG_ERROR("%s: Hart isn't halted before single step!", target->cmd_name);
		return ERROR_TARGET_NOT_HALTED;
	}

	riscv_invalidate_register_cache(target);
	assert(r->on_step);
	r->on_step(target);

	{
		assert(r->step_current_hart);
		int const err = r->step_current_hart(target);
		if (ERROR_OK != err)
			return err;
	}

	riscv_invalidate_register_cache(target);
	{
		assert(r->on_halt);
		int const err = r->on_halt(target);

		if (!riscv_is_halted(target)) {
			LOG_ERROR("%s: Hart was not halted after single step!", target->cmd_name);
			return ERROR_OK != err ? err : ERROR_TARGET_NOT_HALTED;
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
	LOG_DEBUG("%s: debug_reason=%d", target->cmd_name, target->debug_reason);

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

		int result = ERROR_OK;
		for (int i = 0; watchpoint && ERROR_OK == result; ++i) {
			LOG_DEBUG("%s: watchpoint %d: set=%d", target->cmd_name, i, watchpoint->set);
			trigger_temporarily_cleared[i] = watchpoint->set;

			if (watchpoint->set)
				result = target_remove_watchpoint(target, watchpoint);

			watchpoint = watchpoint->next;
		}

		if (ERROR_OK == result)
			result = riscv_step_rtos_hart(target);

		{
			int i = 0;
			for (watchpoint = target->watchpoints; watchpoint; watchpoint = watchpoint->next, ++i) {
				LOG_DEBUG("%s: watchpoint %d: cleared=%d",
					target->cmd_name,
					i,
					trigger_temporarily_cleared[i]);

				if (trigger_temporarily_cleared[i]) {
					int const err = target_add_watchpoint(target, watchpoint);
					result = ERROR_OK == result ? err : result;
				}
			}
		}

		if (ERROR_OK != result)
			return result;
	}

	{
		int const err = riscv_resume_all_harts(target);

		if (ERROR_OK != err) {
			LOG_ERROR("%s: unable to resume all harts", target->cmd_name);
			return err;
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
	LOG_DEBUG("%s: stepping rtos hart", target->cmd_name);

	if (!current) {
		int const err = riscv_set_register(target, GDB_REGNO_PC, address);

		if (ERROR_OK != err)
			return err;
	}

	{
		int const err = riscv_step_rtos_hart(target);

		if (ERROR_OK != err) {
			LOG_ERROR("%s: unable to step rtos hart", target->cmd_name);
			return err;
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
		LOG_ERROR("%s: Command does not take any parameters.", target->cmd_name);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct riscv_info_t *const r = riscv_info(target);
	assert(r);

	if (r->test_compliance) {
		return r->test_compliance(target);
	} else {
		LOG_ERROR("%s: This target does not support this command"
				" (may implement an older version of the spec).",
				target->cmd_name);
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

static void
parse_error(char const *const string,
	char const c,
	unsigned const position)
{
	/** @bug non portable dynamic array */
	char buf[position + 2];
	memset(buf, ' ', position);
	buf[position] = '^';
	buf[position + 1] = 0;

	LOG_ERROR("Parse error at character %c in:" "\n" "%s" "\n" "%s",
			c,
			string,
			buf);
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
					parse_error(argv[0], c, i);
					return ERROR_COMMAND_SYNTAX_ERROR;
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
					parse_error(argv[0], c, i);
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
			}
		}

		if (pass == 0) {
			if (*ranges)
				free(*ranges);
			/** @todo check for free */
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

	struct riscv_info_t *const r = riscv_info(target);

	if (!r) {
		LOG_ERROR("%s: riscv_info is NULL!", target->cmd_name);
		return ERROR_TARGET_INVALID;
	}

	if (r->authdata_read) {
		uint32_t value;

		{
			int const err = r->authdata_read(target, &value);

			if (ERROR_OK != err)
				return err;
		}

		command_print(CMD_CTX, "0x%" PRIx32, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("%s: authdata_read is not implemented for this target.", target->cmd_name);
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

	struct riscv_info_t *const r = riscv_info(target);
	assert(r);

	if (r->authdata_write) {
		return r->authdata_write(target, value);
	} else {
		LOG_ERROR("%s: authdata_write is not implemented for this target.", target->cmd_name);
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

	struct riscv_info_t *const r = riscv_info(target);

	if (!r) {
		LOG_ERROR("%s: riscv_info is NULL!", target->cmd_name);
		return ERROR_TARGET_INVALID;
	}

	if (r->dmi_read) {
		uint32_t address;
		uint32_t value;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

		{
			int const err = r->dmi_read(target, &value, address);

			if (ERROR_OK != err)
				return err;
		}

		command_print(CMD_CTX, "0x%" PRIx32, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("%s: dmi_read is not implemented for this target.", target->cmd_name);
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
	struct riscv_info_t *const r = riscv_info(target);
	assert(r);

	if (r->dmi_write) {
		return r->dmi_write(target, address, value);
	} else {
		LOG_ERROR("%s: dmi_write is not implemented for this target.", target->cmd_name);
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
	struct riscv_info_t *const r = riscv_info(target);
	assert(r);

	if (r->test_sba_config_reg) {
		return
			r->test_sba_config_reg(target, legal_address, num_words, illegal_address, run_sbbusyerror_test);
	} else {
		LOG_ERROR("%s: test_sba_config_reg is not implemented for this target.", target->cmd_name);
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

extern __COMMAND_HANDLER(handle_common_semihosting_command);
extern __COMMAND_HANDLER(handle_common_semihosting_fileio_command);
extern __COMMAND_HANDLER(handle_common_semihosting_resumable_exit_command);
extern __COMMAND_HANDLER(handle_common_semihosting_cmdline);

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

struct target_type riscv_target = {
	.name = "riscv",

	.init_target = riscv_init_target,
	.deinit_target = riscv_deinit_target,
	.examine = riscv_examine,

	/* poll current target status */
	.poll = old_or_new_riscv_poll,

	.halt = old_or_new_riscv_halt,
	.resume = old_or_new_riscv_resume,
	.step = old_or_new_riscv_step,

	.assert_reset = riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = riscv_read_memory,
	.write_memory = riscv_write_memory,

	.checksum_memory = riscv_checksum_memory,

	.get_gdb_reg_list = riscv_get_gdb_reg_list,

	.add_breakpoint = riscv_add_breakpoint,
	.remove_breakpoint = riscv_remove_breakpoint,

	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,
	.hit_watchpoint = riscv_hit_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = riscv_run_algorithm,

	.commands = riscv_command_handlers
};

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

static int
riscv_resume_one_hart(struct target *const target, int const hartid)
{
	LOG_DEBUG("%s: resuming hart %d", target->cmd_name, hartid);

	{
		int const err = riscv_set_current_hartid(target, hartid);

		if (ERROR_OK != err)
			return err;
	}

	if (!riscv_is_halted(target)) {
		LOG_DEBUG("%s:  hart %d requested resume, but was already resumed", target->cmd_name, hartid);
		return ERROR_OK;
	}

	struct riscv_info_t *const r = riscv_info(target);
	assert(r && r->on_resume);
	{
		int const err = r->on_resume(target);

		if (ERROR_OK != err)
			return err;
	}
	assert(r->resume_current_hart);
	return r->resume_current_hart(target);
}

int
riscv_resume_all_harts(struct target *const target)
{
	int const number_of_harts = riscv_count_harts(target);
	int result = ERROR_OK;
	for (int i = 0; i < number_of_harts; ++i)
		if (riscv_hart_enabled(target, i)) {
			int const err = riscv_resume_one_hart(target, i);

			if (ERROR_OK == result && ERROR_OK != err)
				result = err;
		}

	riscv_invalidate_register_cache(target);
	return result;
}

bool
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

	struct riscv_info_t *const r = riscv_info(target);
	assert(r && hartid < RISCV_MAX_HARTS && num <= ('Z' - 'A'));
	return r->harts[hartid].misa & (1 << num);
}

int
riscv_xlen(struct target const *const target)
{
	return riscv_xlen_of_hart(target, riscv_current_hartid(target));
}

int
riscv_xlen_of_hart(struct target const *const target,
	int const hartid)
{
	struct riscv_info_t *const r = riscv_info(target);
	assert(r->harts[hartid].xlen != -1);
	return r->harts[hartid].xlen;
}

bool
riscv_rtos_enabled(const struct target *const target)
{
	return !!target->rtos;
}

/** @return error code */
int
riscv_set_current_hartid(struct target *const target,
	int const hartid)
{
	struct riscv_info_t *const r = riscv_info(target);
	assert(r);

	if (!r->select_current_hart)
		return ERROR_OK;

	int const previous_hartid = riscv_current_hartid(target);
	r->current_hartid = hartid;
	assert(riscv_hart_enabled(target, hartid));
	LOG_DEBUG("%s: setting hartid to %d, was %d", target->cmd_name, hartid, previous_hartid);

	{
		int const err = r->select_current_hart(target);

		if (ERROR_OK != err)
			return err;
	}

	/* This might get called during init, in which case we shouldn't be
	 * setting up the register cache. */
	if (!target_was_examined(target))
		/** @todo ERROR_TARGET_NOT_EXAMINED */
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

	struct riscv_info_t *const r = riscv_info(target);
	assert(r);
	r->registers_initialized = true;
}

int
riscv_current_hartid(const struct target *const target)
{
	struct riscv_info_t *const r = riscv_info(target);
	assert(r);
	return r->current_hartid;
}

void
riscv_set_all_rtos_harts(struct target *const target)
{
	struct riscv_info_t *const r = riscv_info(target);
	assert(r);
	r->rtos_hartid = -1;
}

void
riscv_set_rtos_hartid(struct target *const target, int const hartid)
{
	LOG_DEBUG("%s: setting RTOS hartid %d", target->cmd_name, hartid);
	struct riscv_info_t *const r = riscv_info(target);
	assert(r);
	r->rtos_hartid = hartid;
}

/**
	@return number of HARTs
	@bag return signed value
*/
int
riscv_count_harts(struct target *const target)
{
	/** @bug riscv_count_harts 1 for NULL and bad target */
	if (target == NULL)
		return 1;

	struct riscv_info_t *const r = riscv_info(target);

	if (!r)
		return 1;

	return r->hart_count;
}

/**
	@deprecated Always return true
	@return error code
*/
bool
riscv_has_register(struct target *const target,
	int const hartid,
	int const regid)
{
	return true;
}

/**
	This function is called when the debug user wants to change the value of a
	register. The new value may be cached, and may not be written until the hart
	is resumed.

	@return error code
 */
int
riscv_set_register(struct target *const target,
	enum gdb_regno const r,
	riscv_reg_t const v)
{
	return riscv_set_register_on_hart(target, riscv_current_hartid(target), r, v);
}

/**	@return error code */
int
riscv_set_register_on_hart(struct target *const target,
	int const hartid,
	enum gdb_regno const regid,
	uint64_t const value)
{
	LOG_DEBUG("%s: [%d] %s <- %" PRIx64, target->cmd_name, hartid, gdb_regno_name(regid), value);
	struct riscv_info_t *const r = riscv_info(target);
	assert(r && r->set_register);
	return r->set_register(target, hartid, regid, value);
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

/**
	@param[out] value
	@return error code
*/
int
riscv_get_register_on_hart(struct target *const target,
	riscv_reg_t *const value,
	int const hartid,
	enum gdb_regno const regid)
{
	struct riscv_info_t *const r = riscv_info(target);

	if (riscv_current_hartid(target) != hartid)
		riscv_invalidate_register_cache(target);

	int const err = r->get_register(target, value, hartid, regid);

	if (riscv_current_hartid(target) != hartid)
		riscv_invalidate_register_cache(target);

	assert(value);
	LOG_DEBUG("%s: [%d] %s: %" PRIx64, target->cmd_name, hartid, gdb_regno_name(regid), *value);
	return err;
}

bool
riscv_is_halted(struct target *const target)
{
	struct riscv_info_t *const r = riscv_info(target);
	assert(r && r->is_halted);
	return r->is_halted(target);
}

enum riscv_halt_reason
	riscv_halt_reason(struct target *const target, int hartid)
{
	struct riscv_info_t *const r = riscv_info(target);

	if (riscv_set_current_hartid(target, hartid) != ERROR_OK)
		return RISCV_HALT_ERROR;

	if (!riscv_is_halted(target)) {
		LOG_ERROR("%s: Hart is not halted!", target->cmd_name);
		return RISCV_HALT_UNKNOWN;
	}

	return r->halt_reason(target);
}

bool riscv_hart_enabled(struct target *const target, int hartid)
{
	/* FIXME: Add a hart mask to the RTOS. */
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
int
riscv_enumerate_triggers(struct target *const target)
{
	struct riscv_info_t *const r = riscv_info(target);
	assert(r);

	if (r->triggers_enumerated)
		return ERROR_OK;

	r->triggers_enumerated = true;	/* At the very least we tried. */

	for (int hartid = 0; hartid < riscv_count_harts(target); ++hartid) {
		if (!riscv_hart_enabled(target, hartid))
			continue;

		riscv_reg_t tselect;
		{
			int const err =
				riscv_get_register_on_hart(target, &tselect, hartid, GDB_REGNO_TSELECT);

			if (ERROR_OK != err)
				return err;
		}

		for (unsigned t = 0; t < RISCV_MAX_TRIGGERS; ++t) {
			r->harts[hartid].trigger_count = t;

			riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, t);
			uint64_t tselect_rb;
			{
				int const err =
					riscv_get_register_on_hart(target, &tselect_rb, hartid, GDB_REGNO_TSELECT);
				if (ERROR_OK != err)
					return err;
			}

			/* Mask off the top bit, which is used as tdrmode in old implementations. */
			tselect_rb &= ~(1ULL << (riscv_xlen(target)-1));

			if (tselect_rb != t)
				break;

			uint64_t tdata1;
			{
				int const err =
					riscv_get_register_on_hart(target, &tdata1, hartid, GDB_REGNO_TDATA1);

				if (ERROR_OK != err)
					return err;
			}

			int const type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));

			switch (type) {
				case 1:
					{
						/* On these older cores we don't support software using triggers. */
						int const err = riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);

						if (ERROR_OK != err)
							return err;
					}
					break;

				case 2:
					if (tdata1 & MCONTROL_DMODE(riscv_xlen(target))) {
						int const err = riscv_set_register_on_hart(target, hartid, GDB_REGNO_TDATA1, 0);

						if (ERROR_OK != err)
							return err;
					}
					break;

					/** @bug no default*/
			}
		}

		int const err =
			riscv_set_register_on_hart(target, hartid, GDB_REGNO_TSELECT, tselect);

		if (ERROR_OK != err)
			return err;

		LOG_INFO("%s: [%d] Found %d triggers",
			target->cmd_name, hartid, r->harts[hartid].trigger_count);
	}

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
				else if (regno >= GDB_REGNO_CSR0 && regno <= GDB_REGNO_CSR4095)
					snprintf(buf, sizeof buf - 1, "csr%d", regno - GDB_REGNO_CSR0);
				else if (regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31)
					snprintf(buf, sizeof buf - 1, "f%d", regno - GDB_REGNO_FPR0);
				else
					snprintf(buf, sizeof buf - 1, "gdb_regno_%d", regno);

				return buf;
			}
	}
}

static int
register_get(struct reg *const reg)
{
	assert(reg);
	riscv_reg_info_t *reg_info = reg->arch_info;
	assert(reg_info);
	struct target *const target = reg_info->target;
	uint64_t value;
	{
		int const err = riscv_get_register(target, &value, reg->number);

		if (ERROR_OK != err)
			return err;
	}

	buf_set_u64(reg->value, 0, reg->size, value);
	return ERROR_OK;
}

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
	LOG_DEBUG("%s: write 0x%" PRIx64 " to %s", target->cmd_name, value, reg->name);
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

static int
cmp_csr_info(void const *p1, void const *p2)
{
	return
		(int)((struct csr_info *)p1)->number -
		(int)((struct csr_info *)p2)->number;
}

int
riscv_init_registers(struct target *const target)
{
	struct riscv_info_t *const info = riscv_info(target);

	if (target->reg_cache) {
		if (target->reg_cache->reg_list)
			free(target->reg_cache->reg_list);

		free(target->reg_cache);
	}

	target->reg_cache = calloc(1, sizeof(*target->reg_cache));
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
			target->cmd_name,
			target->reg_cache->num_regs);

	target->reg_cache->reg_list =
		calloc(target->reg_cache->num_regs, sizeof(struct reg));
	assert(target->reg_cache->reg_list);

	static unsigned const max_reg_name_len = 12;
	if (info->reg_names)
		free(info->reg_names);

	info->reg_names =
		calloc(target->reg_cache->num_regs, max_reg_name_len);
	assert(info->reg_names);
	char *reg_name = info->reg_names;

	static struct reg_feature const feature_cpu = {
		.name = "org.gnu.gdb.riscv.cpu"
	};

	static struct reg_feature const feature_fpu = {
		.name = "org.gnu.gdb.riscv.fpu"
	};

	static struct reg_feature const feature_csr = {
		.name = "org.gnu.gdb.riscv.csr"
	};

	static struct reg_feature const feature_virtual = {
		.name = "org.gnu.gdb.riscv.virtual"
	};

	static struct reg_feature const feature_custom = {
		.name = "org.gnu.gdb.riscv.custom"
	};

	static struct reg_data_type const type_ieee_single = {
		.type = REG_TYPE_IEEE_SINGLE,
		.id = "ieee_single"
	};

	static struct reg_data_type const type_ieee_double = {
		.type = REG_TYPE_IEEE_DOUBLE,
		.id = "ieee_double"
	};

	static struct csr_info csr_info[] = {
#define DECLARE_CSR(name, number) { number, #name },
#include "encoding.h"
#undef DECLARE_CSR
	};

	/* encoding.h does not contain the registers in sorted order. */
	qsort(csr_info, DIM(csr_info), sizeof(*csr_info), cmp_csr_info);
	unsigned csr_info_index = 0;

	unsigned custom_range_index = 0;
	int custom_within_range = 0;

	riscv_reg_info_t *const shared_reg_info = calloc(1, sizeof(riscv_reg_info_t));
	assert(shared_reg_info);
	shared_reg_info->target = target;

	/* When gdb requests register N, gdb_get_register_packet() assumes that this
	 * is register at index N in reg_list. So if there are certain registers
	 * that don't exist, we need to leave holes in the list (or renumber, but
	 * it would be nice not to have yet another set of numbers to translate
	 * between). */
	for (uint32_t number = 0; number < target->reg_cache->num_regs; ++number) {
		assert(target && target->reg_cache && target->reg_cache->reg_list && number < target->reg_cache->num_regs);
		struct reg *const r = &target->reg_cache->reg_list[number];
		r->dirty = false;
		r->valid = false;
		r->exist = true;
		r->type = &riscv_reg_arch_type;
		r->arch_info = shared_reg_info;
		r->number = number;
		r->size = riscv_xlen(target);

		/* r->size is set in riscv_invalidate_register_cache, maybe because the
		 * target is in theory allowed to change XLEN on us. But I expect a lot
		 * of other things to break in that case as well. */
		if (number <= GDB_REGNO_XPR31) {
			r->caller_save = true;

			switch (number) {
				case GDB_REGNO_ZERO:
					r->name = "zero";
					break;

				case GDB_REGNO_RA:
					r->name = "ra";
					break;

				case GDB_REGNO_SP:
					r->name = "sp";
					break;

				case GDB_REGNO_GP:
					r->name = "gp";
					break;

				case GDB_REGNO_TP:
					r->name = "tp";
					break;

				case GDB_REGNO_T0:
					r->name = "t0";
					break;

				case GDB_REGNO_T1:
					r->name = "t1";
					break;

				case GDB_REGNO_T2:
					r->name = "t2";
					break;

				case GDB_REGNO_FP:
					r->name = "fp";
					break;

				case GDB_REGNO_S1:
					r->name = "s1";
					break;

				case GDB_REGNO_A0:
					r->name = "a0";
					break;

				case GDB_REGNO_A1:
					r->name = "a1";
					break;

				case GDB_REGNO_A2:
					r->name = "a2";
					break;

				case GDB_REGNO_A3:
					r->name = "a3";
					break;

				case GDB_REGNO_A4:
					r->name = "a4";
					break;

				case GDB_REGNO_A5:
					r->name = "a5";
					break;

				case GDB_REGNO_A6:
					r->name = "a6";
					break;

				case GDB_REGNO_A7:
					r->name = "a7";
					break;

				case GDB_REGNO_S2:
					r->name = "s2";
					break;

				case GDB_REGNO_S3:
					r->name = "s3";
					break;

				case GDB_REGNO_S4:
					r->name = "s4";
					break;

				case GDB_REGNO_S5:
					r->name = "s5";
					break;

				case GDB_REGNO_S6:
					r->name = "s6";
					break;

				case GDB_REGNO_S7:
					r->name = "s7";
					break;

				case GDB_REGNO_S8:
					r->name = "s8";
					break;

				case GDB_REGNO_S9:
					r->name = "s9";
					break;

				case GDB_REGNO_S10:
					r->name = "s10";
					break;

				case GDB_REGNO_S11:
					r->name = "s11";
					break;

				case GDB_REGNO_T3:
					r->name = "t3";
					break;

				case GDB_REGNO_T4:
					r->name = "t4";
					break;

				case GDB_REGNO_T5:
					r->name = "t5";
					break;

				case GDB_REGNO_T6:
					r->name = "t6";
					break;
			}

			r->group = "general";
			/** @todo This should probably be const. */
			r->feature = (struct reg_feature *)&feature_cpu;
		} else if (number == GDB_REGNO_PC) {
			r->caller_save = true;
			reg_name[max_reg_name_len - 1] = '\0';
			snprintf(reg_name, max_reg_name_len - 1, "pc");
			r->group = "general";
			/** @todo This should probably be const. */
			r->feature = (struct reg_feature *)&feature_cpu;
		} else if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
			r->caller_save = true;

			if (riscv_supports_extension(target, riscv_current_hartid(target), 'D')) {
				/** @todo This should probably be const. */
				r->reg_data_type = (struct reg_data_type *)&type_ieee_double;
				r->size = 64;
			} else if (riscv_supports_extension(target, riscv_current_hartid(target), 'F')) {
				/** @todo This should probably be const. */
				r->reg_data_type = (struct reg_data_type *)&type_ieee_single;
				r->size = 32;
			} else {
				r->exist = false;
			}

			switch (number) {
				case GDB_REGNO_FT0:
					r->name = "ft0";
					break;

				case GDB_REGNO_FT1:
					r->name = "ft1";
					break;

				case GDB_REGNO_FT2:
					r->name = "ft2";
					break;

				case GDB_REGNO_FT3:
					r->name = "ft3";
					break;

				case GDB_REGNO_FT4:
					r->name = "ft4";
					break;

				case GDB_REGNO_FT5:
					r->name = "ft5";
					break;

				case GDB_REGNO_FT6:
					r->name = "ft6";
					break;

				case GDB_REGNO_FT7:
					r->name = "ft7";
					break;

				case GDB_REGNO_FS0:
					r->name = "fs0";
					break;

				case GDB_REGNO_FS1:
					r->name = "fs1";
					break;

				case GDB_REGNO_FA0:
					r->name = "fa0";
					break;

				case GDB_REGNO_FA1:
					r->name = "fa1";
					break;

				case GDB_REGNO_FA2:
					r->name = "fa2";
					break;

				case GDB_REGNO_FA3:
					r->name = "fa3";
					break;

				case GDB_REGNO_FA4:
					r->name = "fa4";
					break;

				case GDB_REGNO_FA5:
					r->name = "fa5";
					break;

				case GDB_REGNO_FA6:
					r->name = "fa6";
					break;

				case GDB_REGNO_FA7:
					r->name = "fa7";
					break;

				case GDB_REGNO_FS2:
					r->name = "fs2";
					break;

				case GDB_REGNO_FS3:
					r->name = "fs3";
					break;

				case GDB_REGNO_FS4:
					r->name = "fs4";
					break;

				case GDB_REGNO_FS5:
					r->name = "fs5";
					break;

				case GDB_REGNO_FS6:
					r->name = "fs6";
					break;

				case GDB_REGNO_FS7:
					r->name = "fs7";
					break;

				case GDB_REGNO_FS8:
					r->name = "fs8";
					break;

				case GDB_REGNO_FS9:
					r->name = "fs9";
					break;

				case GDB_REGNO_FS10:
					r->name = "fs10";
					break;

				case GDB_REGNO_FS11:
					r->name = "fs11";
					break;

				case GDB_REGNO_FT8:
					r->name = "ft8";
					break;

				case GDB_REGNO_FT9:
					r->name = "ft9";
					break;

				case GDB_REGNO_FT10:
					r->name = "ft10";
					break;

				case GDB_REGNO_FT11:
					r->name = "ft11";
					break;

			}

			r->group = "float";
			/** @todo This should probably be const. */
			r->feature = (struct reg_feature *)&feature_fpu;
		} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
			r->group = "csr";
			/** @todo This should probably be const. */
			r->feature = (struct reg_feature *)&feature_csr;
			unsigned csr_number = number - GDB_REGNO_CSR0;

			while (csr_info[csr_info_index].number < csr_number && csr_info_index < DIM(csr_info) - 1)
				++csr_info_index;

			if (csr_info[csr_info_index].number == csr_number) {
				r->name = csr_info[csr_info_index].name;
			} else {
				reg_name[max_reg_name_len - 1] = '\0';
				snprintf(reg_name, max_reg_name_len - 1, "csr%d", csr_number);
				/* Assume unnamed registers don't exist, unless we have some
				 * configuration that tells us otherwise. That's important
				 * because eg. Eclipse crashes if a target has too many
				 * registers, and apparently has no way of only showing a
				 * subset of registers in any case. */
				r->exist = false;
			}

			switch (csr_number) {
				case CSR_FFLAGS:
				case CSR_FRM:
				case CSR_FCSR:
					r->exist =
						riscv_supports_extension(target, riscv_current_hartid(target), 'F');
					r->group = "float";
					/** @todo This should probably be const. */
					r->feature = (struct reg_feature *)&feature_fpu;
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
					r->exist = riscv_supports_extension(target,
							riscv_current_hartid(target), 'S');
					break;

				case CSR_MEDELEG:
				case CSR_MIDELEG:
					/* "In systems with only M-mode, or with both M-mode and
					 * U-mode but without U-mode trap support, the medeleg and
					 * mideleg registers should not exist." */
					r->exist = riscv_supports_extension(target, riscv_current_hartid(target), 'S') ||
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
					r->exist = riscv_xlen(target) == 32;
					break;
			}

			if (!r->exist && expose_csr) {
				for (unsigned i = 0; expose_csr[i].low <= expose_csr[i].high; ++i) {
					if (csr_number >= expose_csr[i].low && csr_number <= expose_csr[i].high) {
						LOG_INFO("%s: Exposing additional CSR %d", target->cmd_name, csr_number);
						r->exist = true;
						break;
					}
				}
			}

		} else if (number == GDB_REGNO_PRIV) {
			reg_name[max_reg_name_len - 1] = '\0';
			snprintf(reg_name, max_reg_name_len - 1, "priv");
			r->group = "general";
			/** @todo This should probably be const. */
			r->feature = (struct reg_feature *)&feature_virtual;
			r->size = 8;

		} else {
			/* Custom registers. */
			assert(expose_custom);

			range_t *range = &expose_custom[custom_range_index];
			assert(range->low <= range->high);
			unsigned custom_number = range->low + custom_within_range;

			r->group = "custom";
			/** @todo This should probably be const. */
			r->feature = (struct reg_feature *)&feature_custom;
			r->arch_info = calloc(1, sizeof(riscv_reg_info_t));
			assert(r->arch_info);
			((riscv_reg_info_t *)r->arch_info)->target = target;
			((riscv_reg_info_t *)r->arch_info)->custom_number = custom_number;
			reg_name[max_reg_name_len - 1] = '\0';
			snprintf(reg_name, max_reg_name_len - 1, "custom%d", custom_number);

			++custom_within_range;
			if (custom_within_range > range->high - range->low) {
				custom_within_range = 0;
				++custom_range_index;
			}
		}

		if (reg_name[0])
			r->name = reg_name;

		reg_name += strlen(reg_name) + 1;
		assert(reg_name < info->reg_names + target->reg_cache->num_regs * max_reg_name_len);
		r->value = &info->reg_cache_values[number];
	}

	return ERROR_OK;
}
