#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/target_type.h"
#include "target/breakpoints.h"
#include "target/register.h"
#include "helper/log.h"
#include "helper/binarybuffer.h"
#include "helper/types.h"
#include "jtag/jtag.h"

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <memory.h>

#define IR_SELECT_USING_CACHE 1
#define DAP_CONTROL_USING_CACHE 1
#define VERIFY_DAP_CONTROL 0
#define VERIFY_HART_REGTRANS_WRITE 0
#define VERIFY_CORE_REGTRANS_WRITE 0

#define LOCAL_CONCAT(x,y) x##y
// #define STATIC_ASSERT(e) typedef char LOCAL_CONCAT(___my_static_assert,__LINE__)[1 - 2 * !(e)]
#define STATIC_ASSERT(e) {enum {___my_static_assert = 1 / (!!(e)) };}
#define ARRAY_LEN(arr) (sizeof (arr) / sizeof (arr)[0])
#define BIT_NUM_TO_MASK(bit_num) (1u << (bit_num))
#define LOW_BITS_MASK(n) (~(~0 << (n)))
#define NUM_BITS_TO_SIZE(num_bits) ( ( (size_t)(num_bits) + (8 - 1) ) / 8 )

#define zero 0u
#define sp 14u

typedef uint32_t instr_type;
#define EXTRACT_FIELD(bits, first_bit, last_bit) (((bits) >> (first_bit)) & LOW_BITS_MASK((last_bit) + 1u - (first_bit)))
#define MAKE_FIELD(bits, first_bit, last_bit)     (((instr_type)(bits) & LOW_BITS_MASK((last_bit) + 1u - (first_bit))) << (first_bit))

#define RV_INSTR_R_TYPE(func7, rs2, rs1, func3, rd, opcode) ( \
    MAKE_FIELD((func7), 25, 31) | \
    MAKE_FIELD((rs2),   20, 24) | \
    MAKE_FIELD((rs1),   15, 19) | \
    MAKE_FIELD((func3), 12, 14) | \
    MAKE_FIELD((rd),     7, 11) | \
    MAKE_FIELD((opcode), 0,  6))

#define RV_INSTR_I_TYPE(imm, rs1, func3, rd, opcode) ( \
    MAKE_FIELD((imm),   20, 31) | \
    MAKE_FIELD((rs1),   15, 19) | \
    MAKE_FIELD((func3), 12, 14) | \
    MAKE_FIELD((rd),     7, 11) | \
    MAKE_FIELD((opcode), 0,  6))

#define RV_INSTR_S_TYPE(imm, rs2, rs1, func3, opcode) ( \
    MAKE_FIELD(EXTRACT_FIELD((imm), 5, 11), 25, 31) | \
    MAKE_FIELD((rs2),                                20, 24) | \
    MAKE_FIELD((rs1),                                15, 19) | \
    MAKE_FIELD((func3),                              12, 14) | \
    MAKE_FIELD(EXTRACT_FIELD((imm), 0,  4),  7, 11) | \
    MAKE_FIELD((opcode),                              0,  6))

#define RV_INSTR_SB_TYPE(imm, rs2, rs1, func3, opcode) ( \
    MAKE_FIELD(EXTRACT_FIELD((imm), 12, 12), 31, 31) | \
    MAKE_FIELD(EXTRACT_FIELD((imm),  5, 10), 25, 30) | \
    MAKE_FIELD((rs2),                                 20, 24) | \
    MAKE_FIELD((rs1),                                 15, 19) | \
    MAKE_FIELD((func3),                               12, 14) | \
    MAKE_FIELD(EXTRACT_FIELD((imm),  1,  4),  8, 11) | \
    MAKE_FIELD(EXTRACT_FIELD((imm), 11, 11),  7,  7) | \
    MAKE_FIELD((opcode),                               0,  6))

#define RV_INSTR_U_TYPE(imm, rd, opcode) ( \
    MAKE_FIELD(EXTRACT_FIELD((imm), 12, 31), 12, 31) | \
    MAKE_FIELD((rd),                                   7, 11) | \
    MAKE_FIELD((opcode),                               0,  6))

#define RV_INSTR_UJ_TYPE(imm, rd, opcode) ( \
    MAKE_FIELD(EXTRACT_FIELD((imm), 20, 20), 31, 31) | \
    MAKE_FIELD(EXTRACT_FIELD((imm),  1, 10), 21, 30) | \
    MAKE_FIELD(EXTRACT_FIELD((imm), 11, 11), 20, 20) | \
    MAKE_FIELD(EXTRACT_FIELD((imm), 12, 19), 12, 19) | \
    MAKE_FIELD((rd),                                   7, 11) | \
    MAKE_FIELD((opcode),                               0,  6))

#define RV_ADD(rd, rs1, rs2) RV_INSTR_R_TYPE(0u, rs2, rs1, 0u, rd, 0x33u)
#define RV_ADDI(rd, rs1, imm) RV_INSTR_I_TYPE(imm, rs1, 0, rd, 0x13)
#define RV_NOP() RV_ADDI(zero, zero, 0u)
#define RV_SBREAK() RV_INSTR_I_TYPE(1u, 0u, 0u, 0u, 0x73u)
#define RV_LB(rd, base, imm) RV_INSTR_I_TYPE(imm, base, 0u, rd, 3u)
#define RV_LH(rd, base, imm) RV_INSTR_I_TYPE(imm, base, 1u, rd, 3u)
#define RV_LW(rd, base, imm) RV_INSTR_I_TYPE(imm, base, 2u, rd, 3u)
#define RV_LBU(rd, base, imm) RV_INSTR_I_TYPE(imm, base, 4u, rd, 3u)
#define RV_LHU(rd, base, imm) RV_INSTR_I_TYPE(imm, base, 5u, rd, 3u)

#define RV_SB(rs, base, imm) RV_INSTR_S_TYPE(imm, rs, base, 0u, 0x23)
#define RV_SH(rs, base, imm) RV_INSTR_S_TYPE(imm, rs, base, 1u, 0x23)
#define RV_SW(rs, base, imm) RV_INSTR_S_TYPE(imm, rs, base, 2u, 0x23)
#define RV_AUIPC(rd, imm) RV_INSTR_U_TYPE(imm, rd, 0x17u)
#define RV_CSRRW(rd, csr, rs1) RV_INSTR_I_TYPE((csr), rs1, 1u, rd, 0x73u)
#define RV_JAL(rd, imm) RV_INSTR_UJ_TYPE(imm, rd, 0x6Fu)
#define RV_JALR(rd, rs1, imm) RV_INSTR_I_TYPE(imm, rs1, 0u, rd, 0x67u)
#define RV_FMV_X_S(rd, rs1) RV_INSTR_R_TYPE(0x70u, 0u, rs1, 0u, rd, 0x53u)
#define RV_FMV_S_X(rd, rs1) RV_INSTR_R_TYPE(0x78u, 0u, rs1, 0u, rd, 0x53u)

#define DAP_OPSTATUS_MASK (BIT_NUM_TO_MASK(DAP_OPSTATUS_EXCEPT) | BIT_NUM_TO_MASK(DAP_OPSTATUS_ERROR) | BIT_NUM_TO_MASK(DAP_OPSTATUS_LOCK) | BIT_NUM_TO_MASK(DAP_OPSTATUS_READY))
#define DAP_OPSTATUS_OK (BIT_NUM_TO_MASK(DAP_OPSTATUS_READY))

enum
{
	XLEN = 32u,
	ILEN = 32u,
	FLEN = 32u,
	REG_PC_NUMBER = 32u,
	TOTAL_NUMBER_OF_REGS = 32 + 1 + 32,
};

enum
{
	CSR_DBG_SCRATCH = 0x788u
};

enum TAP_IR_e
{
	TAP_INSTR_DBG_ID = 3,
	TAP_INSTR_BLD_ID = 4,
	TAP_INSTR_DBG_STATUS = 5,
	TAP_INSTR_DAP_CTRL = 6,
	TAP_INSTR_DAP_CTRL_RD = 7,
	TAP_INSTR_DAP_CMD = 8,
	TAP_INSTR_IDCODE = 0xE,  ///< recommended
	TAP_INSTR_BYPASS = 0xF,  ///< mandatory
};

enum TAP_DR_LEN_e
{
	TAP_IR_LEN = 4,
	TAP_LEN_IDCODE = 32,  ///< mandatory
	TAP_LEN_DBG_ID = 32,
	TAP_LEN_BLD_ID = 32,
	TAP_LEN_DBG_STATUS = 32,
	TAP_LEN_DAP_CTRL_UNIT = 2,
	TAP_LEN_DAP_CTRL_FGROUP = 2,
	TAP_LEN_DAP_CTRL = TAP_LEN_DAP_CTRL_UNIT + TAP_LEN_DAP_CTRL_FGROUP,
	TAP_LEN_DAP_CMD_OPCODE = 4,
	TAP_LEN_DAP_CMD_OPCODE_EXT = 32,
	TAP_LEN_DAP_CMD = TAP_LEN_DAP_CMD_OPCODE + TAP_LEN_DAP_CMD_OPCODE_EXT,
	TAP_LEN_BYPASS = 1,  ///< mandatory
};

/// @see TAP_INSTR_DBG_STATUS
enum type_dbgc_core_dbg_sts_reg_bits_e
{
	DBGC_CORE_CDSR_HART0_DMODE_BIT = 0,
	DBGC_CORE_CDSR_HART0_RST_BIT = 1,
	DBGC_CORE_CDSR_HART0_ERR_BIT = 3,
	DBGC_CORE_CDSR_LOCK_BIT = 30,
	DBGC_CORE_CDSR_READY_BIT = 31,
};

/// @see TAP_INSTR_DAP_CMD
/// @see TAP_INSTR_DAP_CTRL
enum DAP_OPSTATUS_BITS_e
{
	DAP_OPSTATUS_EXCEPT = 0,
	DAP_OPSTATUS_ERROR = 1,
	DAP_OPSTATUS_LOCK = 2,
	DAP_OPSTATUS_READY = 3,
};

/// Units IDs
enum type_dbgc_unit_id_e
{
	DBGC_UNIT_ID_HART_0 = 0,
	DBGC_UNIT_ID_HART_1 = 1,
	DBGC_UNIT_ID_CORE = 3,
};

/// Functional groups for HART units
///@{
enum type_dbgc_hart_fgroup_e
{
	/// @see type_dbgc_regblock_hart_e
	DBGC_FGRP_HART_REGTRANS = 0,

	/// @see type_dbgc_dap_cmd_opcode_dbgcmd_e
	DBGC_FGRP_HART_DBGCMD = 1,
};

/// @see DBGC_FGRP_HART_REGTRANS
enum type_dbgc_regblock_hart_e
{
	/// Hart Debug Control Register (HART_DBG_CTRL, HDCR)
	/// @see type_dbgc_hart_dbg_ctrl_reg_bits_e
	DBGC_HART_REGS_DBG_CTRL = 0,

	/// Hart Debug Status Register (HART_DBG_STS, HDSR) 
	/// @see type_dbgc_hart_dbg_sts_reg_bits_e
	DBGC_HART_REGS_DBG_STS = 1,  

	/// Hart Debug Mode Enable Register (HART_DMODE_ENBL, HDMER) 
	/// @see type_dbgc_hart_dmode_enbl_reg_bits_e
	DBGC_HART_REGS_DMODE_ENBL = 2,  

	/// Hart Debug Mode Cause Register (HART_DMODE_CAUSE, HDMCR) 
	/// @see type_dbgc_hart_dmode_cause_reg_bits_e
	DBGC_HART_REGS_DMODE_CAUSE = 3,

	DBGC_HART_REGS_CORE_INSTR = 4,
	DBGC_HART_REGS_DBG_DATA = 5,
	DBGC_HART_REGS_PC_SAMPLE = 6,
};

/// @see DBGC_HART_REGS_DBG_CTRL
enum type_dbgc_hart_dbg_ctrl_reg_bits_e
{
	DBGC_HART_HDCR_RST_BIT = 0,
	DBGC_HART_HDCR_PC_ADVMT_DSBL_BIT = 6,
};

/// @see DBGC_HART_REGS_DBG_STS
enum type_dbgc_hart_dbg_sts_reg_bits_e
{
	DBGC_HART_HDSR_DMODE_BIT = 0,
	DBGC_HART_HDSR_RST_BIT = 1,
	DBGC_HART_HDSR_RST_STKY_BIT = 2,
	DBGC_HART_HDSR_EXCEPT_BIT = 3,
	DBGC_HART_HDSR_ERR_BIT = 16,
	DBGC_HART_HDSR_ERR_HWTHREAD_BIT = 17,
	DBGC_HART_HDSR_ERR_DAP_OPCODE_BIT = 18,
	DBGC_HART_HDSR_ERR_DBGCMD_NACK_BIT = 19,
	DBGC_HART_HDSR_LOCK_STKY_BIT = 31
};

/// Hart Debug Mode Enable Register (HART_DMODE_ENBL, HDMER)
/// @see DBGC_HART_REGS_DMODE_ENBL
enum type_dbgc_hart_dmode_enbl_reg_bits_e
{
	DBGC_HART_HDMER_SW_BRKPT_BIT = 3,
	DBGC_HART_HDMER_SINGLE_STEP_BIT = 29,
	DBGC_HART_HDMER_RST_BREAK_BIT = 30,
};

/// Hart Debug Mode Cause Register (HART_DMODE_CAUSE, HDMCR)
/// @see DBGC_HART_REGS_DMODE_CAUSE
enum type_dbgc_hart_dmode_cause_reg_bits_e
{
	DBGC_HART_HDMCR_SW_BRKPT_BIT = 3,
	DBGC_HART_HDMCR_SINGLE_STEP_BIT = 29,
	DBGC_HART_HDMCR_RST_BREAK_BIT = 30,
	DBGC_HART_HDMCR_ENFORCE_BIT = 31
};

/// @see DBGC_FGRP_HART_DBGCMD
enum type_dbgc_dap_cmd_opcode_dbgcmd_e
{
	/// @see type_dbgc_dap_cmd_opcode_dbgctrl_ext_e
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL = 0,
	DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC = 1,
	DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR = 2,
	DBGC_DAP_OPCODE_DBGCMD_UNLOCK = 3,
};

/// @see DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL
/// @see type_dbgc_dap_cmd_opcode_dbgctrl_ext_s
enum type_dbgc_dap_cmd_opcode_dbgctrl_ext_e
{
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_HALT = 0,
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_RESUME = 1,
	DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS = 2,
};

///@}

/// Functional groups for CORE units
///@{
enum type_dbgc_core_fgroup_e
{
	/// @see type_dbgc_regblock_core_e
	DBGC_FGRP_CORE_REGTRANS = 0,
};

/// @see DBGC_FGRP_CORE_REGTRANS
enum type_dbgc_regblock_core_e
{
	DBGC_CORE_REGS_DEBUG_ID = 0,

	/// @see type_dbgc_core_dbg_ctrl_reg_bits_e
	DBGC_CORE_REGS_DBG_CTRL = 1,
	DBGC_CORE_REGS_DBG_STS = 2,
	DBGC_CORE_REGS_DBG_CMD = 3,
};

/// Core Debug Control Register (CORE_DBG_CTRL, CDCR)
/// @see DBGC_CORE_REGS_DBG_CTRL
enum type_dbgc_core_dbg_ctrl_reg_bits_e
{
	DBGC_CORE_CDCR_HART0_RST_BIT = 0,
	DBGC_CORE_CDCR_HART1_RST_BIT = 8,
	DBGC_CORE_CDCR_RST_BIT = 24,
	DBGC_CORE_CDCR_IRQ_DSBL_BIT = 25,
};

///@}

typedef struct reg_cache reg_cache;
typedef struct reg_arch_type reg_arch_type;
typedef struct target_type target_type;
typedef struct scan_field scan_field;
typedef struct target target;
typedef struct reg reg;

struct This_Arch
{
	int error_code;
	uint8_t last_DAP_ctrl;
};
typedef struct This_Arch This_Arch;

/// Error code handling
///@{
static int
error_code__get(target const* const restrict p_target)
{
	assert(p_target);
	This_Arch const* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	return p_arch->error_code;
}

static int
error_code__update(target const* const restrict p_target, int const a_error_code)
{
	assert(p_target);
	This_Arch* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	if (ERROR_OK == error_code__get(p_target) && ERROR_OK != a_error_code) {
		p_arch->error_code = a_error_code;
	}
	return error_code__get(p_target);
}

static int
error_code__clear(target const* const restrict p_target)
{
	assert(p_target);
	This_Arch* const const restrict p_arch = p_target->arch_info;
	assert(p_arch);
	int const result = error_code__get(p_target);
	p_arch->error_code = ERROR_OK;
	return result;
}
///@}

/// TAPs methods
/// @{
static void
IR_select(target const* const restrict p_target, enum TAP_IR_e const new_instr)
{
	assert(p_target);
	assert(p_target->tap);
#if IR_SELECT_USING_CACHE
	if (buf_get_u32(p_target->tap->cur_instr, 0u, p_target->tap->ir_length) == new_instr) {
		LOG_DEBUG("IR %s resently selected %d", p_target->cmd_name, new_instr);
		return;
	}
#endif
	assert(p_target->tap->ir_length == TAP_IR_LEN);
	uint8_t out_buffer[NUM_BITS_TO_SIZE(TAP_IR_LEN)] = {};
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_IR_LEN) == 1u);
	STATIC_ASSERT(sizeof out_buffer == 1u);
	buf_set_u32(out_buffer, 0, TAP_IR_LEN, new_instr);
	scan_field field =
	{
		.num_bits = p_target->tap->ir_length,
		.out_value = out_buffer,
	};
	jtag_add_ir_scan(p_target->tap, &field, TAP_IDLE);
	LOG_DEBUG("irscan %s %d", p_target->cmd_name, new_instr);
	// force jtag_execute_queue() because field referenced local variable out_buffer
	if (error_code__update(p_target, jtag_execute_queue()) != ERROR_OK) {
		LOG_ERROR("Error %d", error_code__get(p_target));
	}
}

static uint32_t
DBG_STATUS_get(target const* const restrict p_target)
{
	assert(p_target);
	assert(p_target->tap);
	IR_select(p_target, TAP_INSTR_DBG_STATUS);
	if (error_code__get(p_target) != ERROR_OK) {
		/// @todo return bad status
		return 0xBADC0DE0u;
	}

	uint8_t result_buffer[NUM_BITS_TO_SIZE(TAP_LEN_DBG_STATUS)] = {};
	scan_field const field =
	{
		.num_bits = TAP_LEN_DBG_STATUS,
		.in_value = result_buffer,
	};
	jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);

	// enforce jtag_execute_queue() to obtain result
	if (error_code__update(p_target, jtag_execute_queue()) != ERROR_OK) {
		LOG_ERROR("JTAG error %d", error_code__get(p_target));
		return 0;
	}

	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DBG_STATUS) <= sizeof(uint32_t));
	uint32_t const result = buf_get_u32(result_buffer, 0, TAP_LEN_DBG_STATUS);
	LOG_DEBUG("drscan %s %d 0 --> %#010x", p_target->cmd_name, field.num_bits, result);

	if ((result & (BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT) | BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT))) != (uint32_t)BIT_NUM_TO_MASK(DBGC_CORE_CDSR_READY_BIT)) {
		LOG_WARNING("TAP_INSTR_DBG_STATUS is %x!", result);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
	}
	return result;
}

static void
DAP_CTRL_REG_set(target const* const restrict p_target, enum type_dbgc_unit_id_e const dap_unit, uint8_t const dap_group)
{
	assert(p_target);
	assert(
		(
		(
		(dap_unit == DBGC_UNIT_ID_HART_0 && 0 == p_target->coreid) ||
		(dap_unit == DBGC_UNIT_ID_HART_1 && 1 == p_target->coreid)
		) && (dap_group == DBGC_FGRP_HART_REGTRANS || dap_group == DBGC_FGRP_HART_DBGCMD)
		) ||
		(dap_unit == DBGC_UNIT_ID_CORE && dap_group == DBGC_FGRP_HART_REGTRANS)
		);

	uint8_t const set_dap_unit_group =
		MAKE_FIELD(
		MAKE_FIELD(dap_unit, TAP_LEN_DAP_CTRL_FGROUP, TAP_LEN_DAP_CTRL_FGROUP + TAP_LEN_DAP_CTRL_UNIT - 1) |
		MAKE_FIELD(dap_group, 0, TAP_LEN_DAP_CTRL_FGROUP - 1),
		0,
		TAP_LEN_DAP_CTRL_FGROUP + TAP_LEN_DAP_CTRL_UNIT - 1);

	This_Arch* const restrict p_arch = p_target->arch_info;
	assert(p_arch);
#if DAP_CONTROL_USING_CACHE
	if (p_arch->last_DAP_ctrl == set_dap_unit_group) {
		LOG_DEBUG("DAP_CTRL_REG of %s already %#03x", p_target->cmd_name, set_dap_unit_group);
		return;
	}
#endif

	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof set_dap_unit_group);
	/// set unit/group
	{
		IR_select(p_target, TAP_INSTR_DAP_CTRL);
		if (error_code__get(p_target) != ERROR_OK) {
			return;
		}
		// clear status bits
		uint8_t status = 0;
		STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof status);
		scan_field const field =
		{
			.num_bits = TAP_LEN_DAP_CTRL,
			.out_value = &set_dap_unit_group,
			.in_value = &status,
		};
		jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
		// enforce jtag_execute_queue() to get status
		if (error_code__update(p_target, jtag_execute_queue()) != ERROR_OK) {
			LOG_ERROR("JTAG error %d", error_code__get(p_target));
			return;
		}
		/// Update cache of DAP control
		p_arch->last_DAP_ctrl = set_dap_unit_group;
		LOG_DEBUG("drscan %s %d %#03x --> %#03x", p_target->cmd_name, field.num_bits, set_dap_unit_group, status);
		if ((status & DAP_OPSTATUS_MASK) != DAP_OPSTATUS_OK) {
			LOG_ERROR("TAP status %#03x", (uint32_t)status);
#if 0
			error_code__update(p_target, ERROR_TARGET_FAILURE);
			return;
#endif
	}
}

#if VERIFY_DAP_CONTROL
	/// verify unit/group
	{
		IR_select(p_target, TAP_INSTR_DAP_CTRL_RD);
		if (error_code__get(p_target) != ERROR_OK) {
			return;
		}
		uint8_t get_dap_unit_group = 0;
		STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CTRL) == sizeof get_dap_unit_group);
		scan_field const field =
		{
			.num_bits = TAP_LEN_DAP_CTRL,
			.in_value = &get_dap_unit_group,
		};
		// enforce jtag_execute_queue() to get get_dap_unit_group
		jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
		if (error_code__update(p_target, jtag_execute_queue()) != ERROR_OK) {
			LOG_ERROR("JTAG error %d", error_code__get(p_target));
			return;
		}
		LOG_DEBUG("drscan %s %d %#03x --> %#03x", p_target->cmd_name, field.num_bits, 0, get_dap_unit_group);
		if (get_dap_unit_group != set_dap_unit_group) {
			LOG_ERROR("Unit/Group verification error: set %#0x, but get %#0x!", set_dap_unit_group, get_dap_unit_group);
			error_code__update(p_target, ERROR_TARGET_FAILURE);
			return;
		}
	}
#endif
}

static inline int
HART_status_bits_to_target_state(target const* const restrict p_target, uint8_t const status)
{
	if (error_code__get(p_target) != ERROR_OK) {
		return TARGET_UNKNOWN;
	}
	if (status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_ERR_BIT)) {
		return TARGET_UNKNOWN;
	} else if (status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_RST_BIT)) {
		return TARGET_RESET;
	} else if (status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_HART0_DMODE_BIT)) {
		return TARGET_HALTED;
	} else {
		return TARGET_RUNNING;
	}
}

static uint32_t
DAP_CMD_scan(target const* const restrict p_target, uint8_t const DAP_OPCODE, uint32_t const DAP_OPCODE_EXT)
{
	assert(p_target);
	IR_select(p_target, TAP_INSTR_DAP_CMD);
	if (error_code__get(p_target) != ERROR_OK) {
		/// @todo return bad data
		return 0xBADC0DE1u;
	}
	// Output fields
	uint8_t const dap_opcode = DAP_OPCODE;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE) == sizeof dap_opcode);

	uint32_t const dap_opcode_ext = DAP_OPCODE_EXT;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE_EXT) == sizeof dap_opcode_ext);

	// Input fields
	uint8_t DAP_OPSTATUS = 0;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE) == sizeof DAP_OPSTATUS);

	uint32_t DBG_DATA = 0;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE_EXT) == sizeof DBG_DATA);

	scan_field const fields[2] =
	{
		[0] = {
			.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,
			.out_value = (uint8_t const*)&dap_opcode_ext,
			.in_value = (uint8_t*)&DBG_DATA,
		},
		[1] =
			{
				.num_bits = TAP_LEN_DAP_CMD_OPCODE,
				.out_value = &dap_opcode,
				.in_value = &DAP_OPSTATUS,
			},
	};

	assert(p_target->tap);
	jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);

	// enforse jtag_execute_queue() to get values
	if (error_code__update(p_target, jtag_execute_queue()) != ERROR_OK) {
		LOG_ERROR("JTAG error %d", error_code__get(p_target));
		return DBG_DATA;
	}

	LOG_DEBUG("drscan %s %d %#010x %d %#03x --> %#010x %#03x", p_target->cmd_name, fields[0].num_bits, dap_opcode_ext, fields[1].num_bits, dap_opcode, DBG_DATA, DAP_OPSTATUS);

	if ((DAP_OPSTATUS & DAP_OPSTATUS_MASK) != DAP_OPSTATUS_OK) {
		LOG_ERROR("DAP_OPSTATUS == %#0x", (uint32_t)DAP_OPSTATUS);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
	}
	return DBG_DATA;
}

static void
unlock(target* const restrict p_target)
{
	LOG_WARNING("Try to unlock!");
	IR_select(p_target, TAP_INSTR_DAP_CTRL);
	uint8_t const set_dap_unit_group = 0x1;
	uint8_t status = 0;
	scan_field const field =
	{
		.num_bits = TAP_LEN_DAP_CTRL,
		.out_value = &set_dap_unit_group,
		.in_value = &status,
	};
	jtag_add_dr_scan(p_target->tap, 1, &field, TAP_IDLE);
	// enforce jtag_execute_queue() to get status
	error_code__update(p_target, jtag_execute_queue());
	if (error_code__get(p_target) != ERROR_OK) {
		LOG_ERROR("JTAG error %d", error_code__get(p_target));
	}
	/// Update cache of DAP control
	This_Arch* p_arch = p_target->arch_info;
	p_arch->last_DAP_ctrl = set_dap_unit_group;

	IR_select(p_target, TAP_INSTR_DAP_CMD);
	// Output fields
	uint8_t const dap_opcode = DBGC_DAP_OPCODE_DBGCMD_UNLOCK;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE) == sizeof dap_opcode);

	uint32_t const dap_opcode_ext = 0xFEEDBEEFu;
	STATIC_ASSERT(NUM_BITS_TO_SIZE(TAP_LEN_DAP_CMD_OPCODE_EXT) == sizeof dap_opcode_ext);

	scan_field const fields[2] =
	{
		[0] = {
			.num_bits = TAP_LEN_DAP_CMD_OPCODE_EXT,
			.out_value = (uint8_t const*)&dap_opcode_ext,
		},
		[1] =
			{
				.num_bits = TAP_LEN_DAP_CMD_OPCODE,
				.out_value = &dap_opcode,
			},
	};

	assert(p_target->tap);
	jtag_add_dr_scan(p_target->tap, ARRAY_LEN(fields), fields, TAP_IDLE);
	// enforse jtag_execute_queue() to get values
	error_code__update(p_target, jtag_execute_queue());
}

static void
update_status(target* const restrict p_target)
{
	assert(p_target);
	/// Only 1 HART available
	assert(p_target->coreid == 0);
	uint32_t const core_status = DBG_STATUS_get(p_target);
	if (0 != (core_status & BIT_NUM_TO_MASK(DBGC_CORE_CDSR_LOCK_BIT))) {
		unlock(p_target);
	}
	uint8_t const HART_status = (core_status >> p_target->coreid) & 0xFFu;
	p_target->state = HART_status_bits_to_target_state(p_target, HART_status);
}

static inline uint8_t
REGTRANS_scan_type(bool const write, uint8_t const index)
{
	assert((index & !LOW_BITS_MASK(3)) == 0);
	return (write ? BIT_NUM_TO_MASK(3) : 0) | index;
}

static void
REGTRANS_write(target const* const restrict p_target, enum type_dbgc_unit_id_e a_unit, uint8_t const a_fgrp, uint8_t const index, uint32_t const data)
{
	assert(p_target);
	DAP_CTRL_REG_set(p_target, a_unit, a_fgrp);
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}
	(void)DAP_CMD_scan(p_target, REGTRANS_scan_type(true, index), data);
}

static uint32_t
REGTRANS_read(target const* const restrict p_target, enum type_dbgc_unit_id_e const a_unit, uint8_t const a_fgrp, uint8_t const index)
{
	assert(p_target);
	DAP_CTRL_REG_set(p_target, a_unit, a_fgrp);
	if (error_code__get(p_target) != ERROR_OK) {
		return 0xBADC0DE0;
	}
	(void)DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0);
	if (error_code__get(p_target) != ERROR_OK) {
		return 0xBADC0DE1;
	}
	return DAP_CMD_scan(p_target, REGTRANS_scan_type(false, index), 0);
}

static inline void
HART_REGTRANS_write(target const* const restrict p_target, enum type_dbgc_regblock_hart_e const index, uint32_t const data)
{
	REGTRANS_write(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_REGTRANS, index, data);
}

static inline uint32_t
HART_REGTRANS_read(target const* const restrict p_target, enum type_dbgc_regblock_hart_e const index)
{
	return REGTRANS_read(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_REGTRANS, index);
}

static inline void
core_REGTRANS_write(target const* const restrict p_target, enum type_dbgc_regblock_core_e const index, uint32_t const data)
{
	REGTRANS_write(p_target, DBGC_UNIT_ID_CORE, DBGC_FGRP_CORE_REGTRANS, index, data);
}

static inline uint32_t
core_REGTRANS_read(target const* const restrict p_target, enum type_dbgc_regblock_core_e const index)
{
	return REGTRANS_read(p_target, DBGC_UNIT_ID_CORE, DBGC_FGRP_CORE_REGTRANS, index);
}
/// @}

static inline void
exec__setup(target const* const restrict p_target)
{
	DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
}

static inline void
exec__set_csr_data(target const* const restrict p_target, uint32_t const csr_data)
{
	DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBGDATA_WR, csr_data);
}

static inline uint32_t
exec__step(target const* const restrict p_target, uint32_t instruction)
{
	return DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_CORE_EXEC, instruction);
}

/// GP registers accessors
///@{
static void
reg_x_operation_conditions_check(reg const* const restrict p_reg)
{
	assert(p_reg);
	assert(!(!p_reg->valid && p_reg->dirty));
	target* p_target = p_reg->arch_info;
	assert(p_target);
	if (!(zero < p_reg->number && p_reg->number < REG_PC_NUMBER)) {
		LOG_WARNING("Bad reg id =%d for register %s", p_reg->number, p_reg->name);
		error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return;
	}
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
	}
}

static int
reg_x_get(reg* const restrict p_reg)
{
	reg_x_operation_conditions_check(p_reg);
	target* p_target = p_reg->arch_info;
	assert(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	if (p_reg->valid) {
		// register cache already valid
		if (p_reg->dirty) {
			LOG_WARNING("Try re-read dirty cache register %s", p_reg->name);
		} else {
			LOG_DEBUG("Try re-read cache register %s", p_reg->name);
		}
	}

	exec__setup(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	int advance_pc_counter = 0;
	// Save p_reg->number register to CSR_DBG_SCRATCH CSR
	exec__step(p_target, RV_CSRRW(zero, CSR_DBG_SCRATCH, p_reg->number));
	advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	// Exec jump back to previous instruction and get saved into CSR_DBG_SCRATCH CSR value
	uint32_t const value = exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
	advance_pc_counter = 0;
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	LOG_DEBUG("Updating cache from register %s <-- %#010x", p_reg->name, value);

	/// update cache value
	buf_set_u32(p_reg->value, 0, XLEN, value);
	p_reg->valid = true;
	p_reg->dirty = false;
	return error_code__clear(p_target);
}

static int
reg_x_set(reg* const restrict p_reg, uint8_t* const restrict buf)
{
	reg_x_operation_conditions_check(p_reg);
	target* p_target = p_reg->arch_info;
	assert(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	uint32_t const value = buf_get_u32(buf, 0, XLEN);
	LOG_DEBUG("Updating register %s <-- %#010x", p_reg->name, value);

#if 0
	if ( p_reg->valid && (buf_get_u32(p_reg->value, 0, XLEN) == value) ) {
		// skip same value
		return error_code__clear(p_target);
}
#endif
	buf_set_u32(p_reg->value, 0, XLEN, value);
	p_reg->valid = true;
	p_reg->dirty = true;

	/// store dirty register to HW
	exec__setup(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	int advance_pc_counter = 0;

	exec__set_csr_data(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	exec__step(p_target, RV_CSRRW(p_reg->number, CSR_DBG_SCRATCH, zero));
	advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	p_reg->dirty = false;
	if (advance_pc_counter != 0) {
		/// Correct pc back after each instruction
		assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
		exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
		advance_pc_counter = 0;
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}
	}
	assert(advance_pc_counter == 0);
	assert(p_reg->valid && !p_reg->dirty);
	return error_code__clear(p_target);
}

static reg_arch_type const reg_x_accessors =
{
	.get = reg_x_get,
	.set = reg_x_set,
};

static int
reg_x0_get(reg* const restrict p_reg)
{
	assert(p_reg);
	memset(p_reg->value, 0, NUM_BITS_TO_SIZE(p_reg->size));
	p_reg->valid = true;
	p_reg->dirty = false;
	return ERROR_OK;
}

static int
reg_x0_set(reg* const restrict p_reg, uint8_t* const restrict buf)
{
	assert(p_reg);
	LOG_ERROR("Try to write to read-only register");
	memset(p_reg->value, 0, NUM_BITS_TO_SIZE(p_reg->size));
	p_reg->valid = true;
	p_reg->dirty = false;
	return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
}

static reg_arch_type const reg_x0_accessors =
{
	.get = reg_x0_get,
	.set = reg_x0_set,
};

static reg*
prepare_temporary_GP_register(target const* const restrict p_target, int const after_reg)
{
	assert(p_target);
	reg_cache const* const p_reg_cache = p_target->reg_cache;
	assert(p_reg_cache);
	reg* const p_reg_list = p_reg_cache->reg_list;
	assert(p_reg_list);
	assert(p_reg_cache->num_regs >= 32);
	reg* p_valid = NULL;
	reg* p_dirty = NULL;
	for (size_t i = after_reg + 1; i < 32; ++i) {
		if (p_reg_list[i].valid) {
			if (p_reg_list[i].dirty) {
				p_dirty = &p_reg_list[i];
				p_valid = p_dirty;
				break;
			} else if (!p_valid) {
				p_valid = &p_reg_list[i];
			}
		}
	}
	if (!p_dirty) {
		if (!p_valid) {
			assert(after_reg + 1 < 32);
			p_valid = &p_reg_list[after_reg + 1];
			if (error_code__update(p_target, reg_x_get(p_valid)) != ERROR_OK) {
				return NULL;
			}
		}
		assert(p_valid);
		assert(p_valid->valid);
		p_valid->dirty = true;
		p_dirty = p_valid;
	}
	assert(p_dirty);
	assert(p_dirty->valid);
	assert(p_dirty->dirty);
	return p_dirty;
}

/// Update pc cache from HW (if non-cached)
static int
reg_pc_get(reg* const restrict p_reg)
{
	assert(p_reg);
	assert(p_reg->number == REG_PC_NUMBER);
	assert(!(!p_reg->valid && p_reg->dirty));

	/// Find temporary GP register
	target* const p_target = p_reg->arch_info;
	assert(p_target);

	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__clear(p_target);
	}

	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
	if (!p_wrk_reg) {
		LOG_ERROR("Temporary GP register not found!");
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return error_code__clear(p_target);
	}

	/// OK, we have temporary register.
	int advance_pc_counter = 0;
	exec__setup(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	/// Copy pc to temporary register by AUIPC instruction
	exec__step(p_target, RV_AUIPC(p_wrk_reg->number, 0));
	advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	/// and store temporary register to CSR_DBG_SCRATCH CSR.
	exec__step(p_target, RV_CSRRW(zero, CSR_DBG_SCRATCH, p_wrk_reg->number));
	advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	/// Correct pc by jump 2 instructions back and get previous command result.
	uint32_t const value = exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
	advance_pc_counter = 0;
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	assert(advance_pc_counter == 0);
	LOG_DEBUG("Updating cache from register %s <-- %#010x", p_reg->name, value);
	// update cached value
	buf_set_u32(p_reg->value, 0, p_reg->size, value);
	p_reg->valid = true;
	p_reg->dirty = false;

	// restore temporary register
	error_code__update(p_target, reg_x_set(p_wrk_reg, p_wrk_reg->value));
	assert(p_wrk_reg->valid && !p_wrk_reg->dirty);
	return error_code__clear(p_target);
}

static int
reg_pc_set(reg* const restrict p_reg, uint8_t* const restrict buf)
{
	assert(p_reg);
	assert(p_reg->number == REG_PC_NUMBER);
	assert(!(!p_reg->valid && p_reg->dirty));
	if (!p_reg->valid) {
		LOG_DEBUG("force rewriting of pc register before read");
	}

	target* const p_target = p_reg->arch_info;
	assert(p_target);
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__clear(p_target);
	}

	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, 0);
	if (!p_wrk_reg) {
		LOG_ERROR("Temporary GP register not found!");
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return error_code__clear(p_target);
	}

	uint32_t const value = buf_get_u32(buf, 0, p_reg->size);
	buf_set_u32(p_reg->value, 0, p_reg->size, value);
	p_reg->valid = true;
	p_reg->dirty = true;
	LOG_DEBUG("Updating register %s <-- %#010x", p_reg->name, value);

	// Update to HW
	exec__setup(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	int advance_pc_counter = 0;

	exec__set_csr_data(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	// set temporary register value to restoring pc value
	exec__step(p_target, RV_CSRRW(p_wrk_reg->number, CSR_DBG_SCRATCH, zero));
	advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	assert(p_reg->dirty);
	assert(p_wrk_reg->dirty);
	/// and exec JARL to set pc
	exec__step(p_target, RV_JALR(zero, p_wrk_reg->number, 0));
	advance_pc_counter = 0;
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	/// OK pc restored
	p_reg->dirty = false;

	// restore temporary register
	error_code__update(p_target, reg_x_set(p_wrk_reg, p_wrk_reg->value));
	assert(p_wrk_reg->valid && !p_wrk_reg->dirty);
	return error_code__clear(p_target);
}

static reg_arch_type const reg_pc_accessors =
{
	.get = reg_pc_get,
	.set = reg_pc_set,
};

static int
reg_f_get(reg* const restrict p_reg)
{
	assert(p_reg);
	assert(REG_PC_NUMBER < p_reg->number && p_reg->number < TOTAL_NUMBER_OF_REGS);
	assert(!(!p_reg->valid && p_reg->dirty));

	target* const p_target = p_reg->arch_info;
	assert(p_target);

	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__clear(p_target);
	}

	/// @todo check that FPU is enabled
	/// Find temporary GP register
	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, zero);
	if (!p_wrk_reg) {
		LOG_ERROR("Temporary GP register not found!");
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return error_code__clear(p_target);
	}

	/// OK, we have temporary register.
	int advance_pc_counter = 0;
	exec__setup(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	/// Copy values to temporary register
	exec__step(p_target, RV_FMV_S_X(p_wrk_reg->number, (p_reg->number - (REG_PC_NUMBER + 1))));
	advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	/// and store temporary register to CSR_DBG_SCRATCH CSR.
	exec__step(p_target, RV_CSRRW(zero, CSR_DBG_SCRATCH, p_wrk_reg->number));
	advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	/// Correct pc by jump 2 instructions back and get previous command result.
	uint32_t const value = exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
	advance_pc_counter = 0;
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	assert(advance_pc_counter == 0);
	LOG_DEBUG("Updating cache from register %s <-- %#010x", p_reg->name, value);
	// update cached value
	buf_set_u32(p_reg->value, 0, p_reg->size, value);
	p_reg->valid = true;
	p_reg->dirty = false;

	// restore temporary register
	error_code__update(p_target, reg_x_set(p_wrk_reg, p_wrk_reg->value));
	assert(p_wrk_reg->valid && !p_wrk_reg->dirty);
	return error_code__clear(p_target);
}

static int
reg_f_set(reg* const restrict p_reg, uint8_t* const restrict buf)
{
	assert(p_reg);
	assert(32 < p_reg->number && p_reg->number < TOTAL_NUMBER_OF_REGS);
	assert(!(!p_reg->valid && p_reg->dirty));

	target* const p_target = p_reg->arch_info;
	assert(p_target);

	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__clear(p_target);
	}

	/// @todo check that FPU is enabled
	/// Find temporary GP register
	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, zero);
	if (!p_wrk_reg) {
		LOG_ERROR("Temporary GP register not found!");
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return error_code__clear(p_target);
	}

	/// OK, we have temporary register.
	int advance_pc_counter = 0;
	exec__setup(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	uint32_t const value = buf_get_u32(buf, 0, p_reg->size);
	LOG_DEBUG("Updating register %s <-- %#010x", p_reg->name, value);

	buf_set_u32(p_reg->value, 0, p_reg->size, value);
	p_reg->valid = true;
	p_reg->dirty = true;

	exec__set_csr_data(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	// set temporary register value to restoring pc value
	exec__step(p_target, RV_CSRRW(p_wrk_reg->number, CSR_DBG_SCRATCH, zero));
	advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	assert(p_reg->dirty);
	assert(p_wrk_reg->dirty);

	assert(zero < p_wrk_reg->number && p_wrk_reg->number < REG_PC_NUMBER);
	exec__step(p_target, RV_FMV_X_S((p_reg->number - (REG_PC_NUMBER + 1)), p_wrk_reg->number));
	advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	/// Correct pc by jump 2 instructions back and get previous command result.
	(void)exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	advance_pc_counter = 0;
	assert(advance_pc_counter == 0);
	/// OK reg saved
	p_reg->dirty = false;

	// restore temporary register
	error_code__update(p_target, reg_x_set(p_wrk_reg, p_wrk_reg->value));
	assert(p_wrk_reg->valid && !p_wrk_reg->dirty);
	return error_code__clear(p_target);
}

static reg_arch_type const reg_f_accessors =
{
	.get = reg_f_get,
	.set = reg_f_set,
};

static reg const reg_def_array[] = {
	// Hard-wired zero
	{.name = "x0", .number = 0, .caller_save = false, .dirty = false, .valid = true, .exist = true, .size = XLEN, .type = &reg_x0_accessors},

	// Return address
	{.name = "x1", .number = 1, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},

	// Stack pointer
	{.name = "x2", .number = 2, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},

	// Global pointer
	{.name = "x3", .number = 3, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},

	// Thread pointer
	{.name = "x4", .number = 4, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},

	// Temporaries
	{.name = "x5", .number = 5, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x6", .number = 6, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x7", .number = 7, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},

	// Saved register/frame pointer
	{.name = "x8", .number = 8, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},

	// Saved register
	{.name = "x9", .number = 9, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},

	// Function arguments/return values
	{.name = "x10", .number = 10, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x11", .number = 11, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},

	// Function arguments
	{.name = "x12", .number = 12, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x13", .number = 13, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x14", .number = 14, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x15", .number = 15, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x16", .number = 16, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x17", .number = 17, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},

	// Saved registers
	{.name = "x18", .number = 18, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x19", .number = 19, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x20", .number = 20, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x21", .number = 21, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x22", .number = 22, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x23", .number = 23, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x24", .number = 24, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x25", .number = 25, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x26", .number = 26, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x27", .number = 27, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},

	// Temporaries
	{.name = "x28", .number = 28, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x29", .number = 29, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x30", .number = 30, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},
	{.name = "x31", .number = 31, .caller_save = true, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_x_accessors},

	// Program counter
	{.name = "pc", .number = 32, .caller_save = false, .dirty = false, .valid = false, .exist = true, .size = XLEN, .type = &reg_pc_accessors},

	// FP temporaries
	{.name = "f0", .number = 33, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f1", .number = 34, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f2", .number = 35, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f3", .number = 36, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f4", .number = 37, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f5", .number = 38, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f6", .number = 39, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f7", .number = 40, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},

	// FP saved registers
	{.name = "f8", .number = 41, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f9", .number = 42, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},

	// FP arguments/return values
	{.name = "f10", .number = 43, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f11", .number = 44, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},

	// FP arguments
	{.name = "f12", .number = 45, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f13", .number = 46, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f14", .number = 47, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f15", .number = 48, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f16", .number = 49, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f17", .number = 50, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},

	// FP saved registers
	{.name = "f18", .number = 51, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f19", .number = 52, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f20", .number = 53, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f21", .number = 54, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f22", .number = 55, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f23", .number = 56, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f24", .number = 57, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f25", .number = 58, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f26", .number = 59, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f27", .number = 60, .caller_save = false, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},

	// FP temporaries
	{.name = "f28", .number = 61, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f29", .number = 62, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f30", .number = 63, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
	{.name = "f31", .number = 64, .caller_save = true, .dirty = false, .valid = false, .exist = false, .size = FLEN, .type = &reg_f_accessors},
};

static reg_cache*
reg_cache__create(char const* name, reg const regs_templates[], size_t const num_regs, void* const p_arch_info)
{
	reg* const p_dst_array = calloc(num_regs, sizeof(reg));
	reg* p_dst_iter = &p_dst_array[0];
	reg const* p_src_iter = &regs_templates[0];
	for (size_t i = 0; i < num_regs; ++i) {
		*p_dst_iter = *p_src_iter;
		p_dst_iter->value = calloc(1, NUM_BITS_TO_SIZE(p_src_iter->size));
		p_dst_iter->arch_info = p_arch_info;

		++p_src_iter;
		++p_dst_iter;
	}
	reg_cache const the_reg_cache = {
		.name = name,
		.reg_list = p_dst_array,
		.num_regs = num_regs,
	};

	reg_cache* const p_obj = calloc(1, sizeof(reg_cache));
	assert(p_obj);
	*p_obj = the_reg_cache;
	return p_obj;
}

static int
this_init_target(struct command_context *cmd_ctx, target* const restrict p_target)
{
	assert(p_target);
	p_target->reg_cache = reg_cache__create("rv32i", reg_def_array, ARRAY_LEN(reg_def_array), p_target);
	This_Arch the_arch = {
		.error_code = ERROR_OK,
		.last_DAP_ctrl = 0xFF,
	};
	This_Arch* p_arch_info = calloc(1, sizeof(This_Arch));
	*p_arch_info = the_arch;

	p_target->arch_info = p_arch_info;
	return ERROR_OK;
}

static void
this_deinit_target(target* const restrict p_target)
{
	assert(p_target);
	This_Arch* const p_arch_info = p_target->arch_info;
	assert(p_arch_info);
	reg_cache* const p_reg_cache = p_target->reg_cache;
	assert(p_reg_cache);
	reg* reg_list = p_reg_cache->reg_list;
	assert(reg_list);
	size_t const num_regs = p_reg_cache->num_regs;
	assert(num_regs == (32 + 32 + 1));
	for (size_t i = 0; i < num_regs; ++i) {
		free(reg_list[i].value);
	}
	free(reg_list);
	free(p_reg_cache);
	free(p_arch_info);
	p_target->arch_info = NULL;
}

static int
this_target_create(target* const restrict p_target, struct Jim_Interp *interp)
{
	return ERROR_OK;
}

static void
regs_invalidate(target const* const restrict p_target)
{
	assert(p_target);
	/// @todo multiple caches
	reg_cache const* const p_reg_cache = p_target->reg_cache;
	assert(p_reg_cache);
	assert(p_reg_cache->num_regs == TOTAL_NUMBER_OF_REGS);
	for (size_t i = 0; i < p_reg_cache->num_regs; ++i) {
		assert(!p_reg_cache->reg_list[i].dirty);
		p_reg_cache->reg_list[i].valid = false;
	}
}

#if 0
static void
regs_commit(target const* const restrict p_target)
{
	assert(p_target);
	/// @todo multiple caches
	reg_cache const* const p_reg_cache = p_target->reg_cache;
	assert(p_reg_cache);

	// pc number == 32
	assert(32 < p_reg_cache->num_regs);
	reg* const restrict p_pc = &p_reg_cache->reg_list[32];

	/// If pc is dirty find first dirty GP register (except zero register)
	reg* const restrict p_tmp_reg = p_pc->dirty ? prepare_temporary_GP_register(p_target, 0) : &p_reg_cache->reg_list[0];

	exec__setup(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}
	/// From last GP register down to first dirty register (or down to zero register)
	int advance_pc_counter = 0;
	for (reg* p_reg = &p_reg_cache->reg_list[31]; p_reg != p_tmp_reg; --p_reg) {
		if (!p_reg->dirty) {
			continue;
		}
		/// store dirty registers to HW
		exec__set_csr_data(p_target, buf_get_u32(p_reg->value, 0, p_reg->size));
		if (error_code__get(p_target) != ERROR_OK) {
			return;
		}
		exec__step(p_target, RV_CSRRW(p_reg->number, DBG_SCRATCH, zero));
		advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
		if (error_code__get(p_target) != ERROR_OK) {
			return;
		}
		/// mark it's values non-dirty
		p_reg->dirty = false;
	}
	if (advance_pc_counter) {
		/// Correct pc back after instructions
		assert(advance_pc_counter % NUM_BITS_TO_SIZE(ILEN) == 0);
		exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
		advance_pc_counter = 0;
		if (error_code__get(p_target) != ERROR_OK) {
			return;
		}
	}

	if (p_tmp_reg == &p_reg_cache->reg_list[0]) {
		/// If first dirty register is zero register, then all already saved
		assert(!p_pc->dirty);
		return;
	}
	/// else
	exec__set_csr_data(p_target, buf_get_u32(p_pc->value, 0, p_pc->size));
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}
	/// set temporary register value to restoring pc value
	exec__step(p_target, RV_CSRRW(p_tmp_reg->number, DBG_SCRATCH, zero));
	advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}

	assert(p_pc->dirty);
	assert(p_tmp_reg->dirty);
	/// and exec JARL to set pc
	exec__step(p_target, RV_JALR(zero, p_tmp_reg->number, 0));
	advance_pc_counter = 0;
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}
	/// OK pc restored
	p_pc->dirty = false;

	/// Restoring temporary register value
	exec__set_csr_data(p_target, buf_get_u32(p_tmp_reg->value, 0, p_tmp_reg->size));
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}
	exec__step(p_target, RV_CSRRW(p_tmp_reg->number, DBG_SCRATCH, zero));
	advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}
	/// and correct pc back
	exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
	advance_pc_counter = 0;
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}
	p_tmp_reg->dirty = false;
	assert(advance_pc_counter == 0);
	}
#endif

static enum target_debug_reason
read_debug_cause(target* const restrict p_target)
{
	uint32_t const value = HART_REGTRANS_read(p_target, DBGC_HART_REGS_DMODE_CAUSE);
	if (error_code__get(p_target) != ERROR_OK) {
		return DBG_REASON_UNDEFINED;
	}
	if (value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_ENFORCE_BIT)) {
		return DBG_REASON_DBGRQ;
	} else if (value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_SINGLE_STEP_BIT)) {
		return DBG_REASON_SINGLESTEP;
	} else if (value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_SW_BRKPT_BIT)) {
		return DBG_REASON_BREAKPOINT;
	} else if (value & BIT_NUM_TO_MASK(DBGC_HART_HDMCR_RST_BREAK_BIT)) {
		return DBG_REASON_DBGRQ;
	} else {
		return DBG_REASON_UNDEFINED;
	}
}

static void
update_debug_reason(target* const restrict p_target)
{
	p_target->debug_reason = read_debug_cause(p_target);
}

static int
this_poll(target* const restrict p_target)
{
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	switch (p_target->state) {
	case TARGET_HALTED:
		target_call_event_callbacks(p_target, TARGET_EVENT_HALTED);
		break;
	case TARGET_RESET:
		target_call_event_callbacks(p_target, TARGET_EVENT_RESET_ASSERT);
		break;
	case TARGET_RUNNING:
	case TARGET_UNKNOWN:
	default:
		break;
	}

	return error_code__clear(p_target);
}

static int
this_arch_state(target* const restrict p_target)
{
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	switch (p_target->state) {
	case TARGET_HALTED:
		update_debug_reason(p_target);
		break;
	default:
		break;
	}

	return error_code__clear(p_target);
}

static int
this_halt(target* const restrict p_target)
{
	assert(p_target);
	{
		// May be already halted?
		// Update state
		update_status(p_target);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		if (p_target->state == TARGET_HALTED) {
			LOG_WARNING("Halt request when RV is already in halted state");
			return error_code__clear(p_target);
		}
	}

	{
		// Try to halt
		DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		(void)DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_HALT) | BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS));
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}
	}

	{
		// update state
		update_status(p_target);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}
	}

	// Verify that in debug mode
	if (p_target->state != TARGET_HALTED) {
		// issue error if we are still running
		LOG_ERROR("RV is not halted after Halt command");
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__clear(p_target);
	}

	// OK, halted
	update_debug_reason(p_target);
	return error_code__clear(p_target);
}

static void
set_DEMODE_ENBL(target* const restrict p_target, uint32_t const set_value)
{
	HART_REGTRANS_write(p_target, DBGC_HART_REGS_DMODE_ENBL, set_value);
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}

#if VERIFY_HART_REGTRANS_WRITE
	uint32_t const get_value = HART_REGTRANS_read(p_target, DBGC_HART_REGS_DMODE_ENBL);
	if (error_code__get(p_target) != ERROR_OK) {
		return;
	}
	if (get_value != set_value) {
		LOG_ERROR("Write DBGC_HART_REGS_DMODE_ENBL with value %#010x, but re-read value is %#010x", set_value, get_value);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return;
	}
#endif
}

static int
common_resume(target* const restrict p_target, uint32_t const dmode_enabled, int const current, uint32_t const address, int const handle_breakpoints, int const debug_execution)
{
	assert(p_target);
	/// @todo update state
	if (p_target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!current) {
		/// @todo multiple caches
		reg_cache* const p_reg_cache = p_target->reg_cache;
		reg* const p_pc = &p_reg_cache->reg_list[32];
		uint8_t buf[sizeof address];
		buf_set_u32(buf, 0, XLEN, address);
		error_code__update(p_target, reg_pc_set(p_pc, buf));
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}
		assert(p_pc->valid);
	}
#if 0
	// upload reg values into HW
	regs_commit(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
#endif
	regs_invalidate(p_target);
	set_DEMODE_ENBL(p_target, dmode_enabled);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	DAP_CTRL_REG_set(p_target, p_target->coreid == 0 ? DBGC_UNIT_ID_HART_0 : DBGC_UNIT_ID_HART_1, DBGC_FGRP_HART_DBGCMD);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	(void)DAP_CMD_scan(p_target, DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL, BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_RESUME) | BIT_NUM_TO_MASK(DBGC_DAP_OPCODE_DBGCMD_DBG_CTRL_CLEAR_STICKY_BITS));
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	// update state
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	target_call_event_callbacks(p_target, debug_execution ? TARGET_EVENT_DEBUG_RESUMED : TARGET_EVENT_RESUMED);

	return error_code__clear(p_target);
		}

static int
this_resume(target* const restrict p_target, int const current, uint32_t const address, int const handle_breakpoints, int const debug_execution)
{
	assert(p_target);
	/// @todo Verify halt
	uint32_t const dmode_enabled = BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT) | BIT_NUM_TO_MASK(DBGC_HART_HDMER_RST_BREAK_BIT);
	return common_resume(p_target, dmode_enabled, current, address, handle_breakpoints, debug_execution);
}

static int
this_step(target* const restrict p_target, int const current, uint32_t const address, int const handle_breakpoints)
{
	assert(p_target);
	/// @todo Verify halt
#if 1
	uint32_t const dmode_enabled = BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT) | BIT_NUM_TO_MASK(DBGC_HART_HDMER_RST_BREAK_BIT) | BIT_NUM_TO_MASK(DBGC_HART_HDMER_SINGLE_STEP_BIT);
#else
	uint32_t const set_value = BIT_NUM_TO_MASK(DBGC_HART_HDMER_SINGLE_STEP_BIT);
#endif
	return common_resume(p_target, dmode_enabled, current, address, handle_breakpoints, false);
}

static int
set_reset_state(target* const restrict p_target, bool const active)
{
	assert(p_target);
	DAP_CTRL_REG_set(p_target, DBGC_UNIT_ID_CORE, DBGC_FGRP_CORE_REGTRANS);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	uint32_t const get_old_value1 = core_REGTRANS_read(p_target, DBGC_CORE_REGS_DBG_CTRL);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	/// @todo replace literals
	static uint32_t const bit_mask = BIT_NUM_TO_MASK(DBGC_CORE_CDCR_HART0_RST_BIT) | BIT_NUM_TO_MASK(DBGC_CORE_CDCR_RST_BIT);

	uint32_t const set_value = (get_old_value1 & ~bit_mask) | (active ? bit_mask : 0u);
	core_REGTRANS_write(p_target, DBGC_CORE_REGS_DBG_CTRL, set_value);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

#if VERIFY_CORE_REGTRANS_WRITE
	{
		// double check
		uint32_t const get_new_value2 = core_REGTRANS_read(p_target, DBGC_CORE_REGS_DBG_CTRL);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}
		if ((get_new_value2 & bit_mask) != (set_value & bit_mask)) {
			LOG_ERROR("Fail to verify reset state: set %#0x, but get %#0x", set_value, get_new_value2);
			error_code__update(p_target, ERROR_TARGET_FAILURE);
			return error_code__clear(p_target);
		}
	}
#endif

	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	if (active) {
		if (p_target->state != TARGET_RESET) {
			/// issue error if we are still running
			LOG_ERROR("RV is not resetting after reset assert");
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		}
	} else {
		if (p_target->state == TARGET_RESET) {
			LOG_ERROR("RV is stiil in reset after reset deassert");
			error_code__update(p_target, ERROR_TARGET_FAILURE);
		}
	}
	return error_code__clear(p_target);
}

static int
this_assert_reset(target* const restrict p_target)
{
	return set_reset_state(p_target, true);
}

static int
this_deassert_reset(target* const restrict p_target)
{
	return set_reset_state(p_target, false);
}


static int
this_soft_reset_halt(target* const restrict p_target)
{
	set_reset_state(p_target, true);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	uint32_t const dmode_enabled = BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT) | BIT_NUM_TO_MASK(DBGC_HART_HDMER_RST_BREAK_BIT);
	set_DEMODE_ENBL(p_target, dmode_enabled);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	set_reset_state(p_target, false);

	return error_code__clear(p_target);
}

static int
this_read_memory(target* const restrict p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t* restrict buffer)
{
	LOG_DEBUG("Read_memory at %#010x, %d items, each %d bytes, total %d bytes", address, count, size, count * size);
	/// Check for size
	if (!(size == 1 || size == 2 || size == 4)) {
		LOG_ERROR("Invalid item size %d", size);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return error_code__clear(p_target);
	}

	/// Check for alignment
	if (address % size != 0) {
		LOG_ERROR("Unaligned access at %#010x, for item size %d", address, size);
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__clear(p_target);
	}

	/// Check that target halted
	{
		update_status(p_target);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}
		if (p_target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
			return error_code__clear(p_target);
		}
	}

	/// Reserve work register
	reg* const p_wrk_reg = prepare_temporary_GP_register(p_target, zero);
	assert(p_wrk_reg);

	/// Define opcode for load item to register
	uint32_t const load_OP =
		size == 4 ? RV_LW(p_wrk_reg->number, p_wrk_reg->number, 0) :
		size == 2 ? RV_LH(p_wrk_reg->number, p_wrk_reg->number, 0) :
		/*size == 1*/RV_LB(p_wrk_reg->number, p_wrk_reg->number, 0);

	/// Setup exec operations mode
	exec__setup(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	int advance_pc_counter = 0;

	/// For count number of items do loop
	while (count--) {
		/// Set address to CSR
		exec__set_csr_data(p_target, address);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		/// Load address to work register
		exec__step(p_target, RV_CSRRW(p_wrk_reg->number, CSR_DBG_SCRATCH, zero));
		advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		/// Exec load item to register
		exec__step(p_target, load_OP);
		advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		/// Exec store work register to csr
		exec__step(p_target, RV_CSRRW(zero, CSR_DBG_SCRATCH, p_wrk_reg->number));
		advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);

		/// get data from csr and jump back to correct pc
		uint32_t const value = exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
		advance_pc_counter = 0;
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		/// store read data to buffer
		buf_set_u32(buffer, 0, 8 * size, value);

		/// advance src/dst pointers
		address += size;
		buffer += size;
	}
	/// end loop
	assert(advance_pc_counter == 0);

	/// restore temporary register
	error_code__update(p_target, reg_x_set(p_wrk_reg, p_wrk_reg->value));
	assert(p_wrk_reg->valid && !p_wrk_reg->dirty);
	return error_code__clear(p_target);
}

static int
this_write_memory(target* const restrict p_target, uint32_t address, uint32_t const size, uint32_t count, uint8_t const* restrict buffer)
{
	LOG_DEBUG("Write_memory at %#010x, %d items, each %d bytes, total %d bytes", address, count, size, count * size);
	/// Check for size
	if (!(size == 1 || size == 2 || size == 4)) {
		LOG_ERROR("Invalid item size %d", size);
		error_code__update(p_target, ERROR_TARGET_FAILURE);
		return error_code__clear(p_target);
	}

	/// Check for alignment
	if (address % size != 0) {
		LOG_ERROR("Unaligned access at %#010x, for item size %d", address, size);
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__clear(p_target);
	}

	/// Check that target halted
	{
		update_status(p_target);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}
		if (p_target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
			return error_code__clear(p_target);
		}
	}
	/// Reserve work register
	reg* const p_addr_reg = prepare_temporary_GP_register(p_target, zero);
	assert(p_addr_reg);
	reg* const p_data_reg = prepare_temporary_GP_register(p_target, p_addr_reg->number);
	assert(p_data_reg);
	assert(p_addr_reg->number != p_addr_reg->number);

	/// Define opcode for load item to register
	uint32_t const store_OP =
		size == 4 ? RV_SW(p_data_reg->number, p_addr_reg->number, 0) :
		size == 2 ? RV_SH(p_data_reg->number, p_addr_reg->number, 0) :
		/*size == 1*/ RV_SB(p_data_reg->number, p_addr_reg->number, 0);

	/// Setup exec operations mode
	exec__setup(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	int advance_pc_counter = 0;

	/// For count number of items do loop
	while (count--) {
		/// Set address to CSR
		exec__set_csr_data(p_target, address);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		/// Load address to work register
		exec__step(p_target, RV_CSRRW(p_addr_reg->number, CSR_DBG_SCRATCH, zero));
		advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		/// Set data to CSR
		exec__set_csr_data(p_target, buf_get_u32(buffer, 0, 8 * size));
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		/// Load data to work register
		exec__step(p_target, RV_CSRRW(p_data_reg->number, CSR_DBG_SCRATCH, zero));
		advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		/// Exec store item from register to memory
		exec__step(p_target, store_OP);
		advance_pc_counter += NUM_BITS_TO_SIZE(ILEN);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		/// jump back to correct pc
		exec__step(p_target, RV_JAL(zero, -advance_pc_counter));
		advance_pc_counter = 0;
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}

		/// advance src/dst pointers
		address += size;
		buffer += size;
	}
	/// end loop

	assert(advance_pc_counter == 0);

	/// restore temporary registers
	error_code__update(p_target, reg_x_set(p_data_reg, p_data_reg->value));
	assert(p_data_reg->valid && !p_data_reg->dirty);
	error_code__update(p_target, reg_x_set(p_addr_reg, p_addr_reg->value));
	assert(p_addr_reg->valid && !p_addr_reg->dirty);

	return error_code__clear(p_target);
}

static int
this_examine(target* const restrict p_target)
{
	if (!target_was_examined(p_target)) {
		uint32_t const dmode_enabled = BIT_NUM_TO_MASK(DBGC_HART_HDMER_SW_BRKPT_BIT) | BIT_NUM_TO_MASK(DBGC_HART_HDMER_RST_BREAK_BIT);
		set_DEMODE_ENBL(p_target, dmode_enabled);
		if (error_code__get(p_target) != ERROR_OK) {
			return error_code__clear(p_target);
		}
	}
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	target_set_examined(p_target);
	return error_code__clear(p_target);
}

static int
this_add_breakpoint(target* const restrict p_target, struct breakpoint* const restrict breakpoint)
{
	assert(breakpoint);
	if (breakpoint->length != NUM_BITS_TO_SIZE(ILEN)) {
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__clear(p_target);
	}

	if ((breakpoint->address % NUM_BITS_TO_SIZE(ILEN)) != 0) {
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__clear(p_target);
	}

	if (breakpoint->type != BKPT_SOFT) {
		error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return error_code__clear(p_target);
	}

	assert(p_target);
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	if (p_target->state != TARGET_HALTED) {
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__clear(p_target);
	}

	error_code__update(p_target, target_read_buffer(p_target, breakpoint->address, breakpoint->length, breakpoint->orig_instr));
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	error_code__update(p_target, target_write_u32(p_target, breakpoint->address, RV_SBREAK()));
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}

	breakpoint->set = 1;

	return error_code__clear(p_target);
}

static int
this_remove_breakpoint(target* const restrict p_target, struct breakpoint* const restrict breakpoint)
{
	assert(breakpoint);
	if (breakpoint->length != NUM_BITS_TO_SIZE(ILEN)) {
		error_code__update(p_target, ERROR_TARGET_INVALID);
		return error_code__clear(p_target);
	}
	if ((breakpoint->address % NUM_BITS_TO_SIZE(ILEN)) != 0) {
		error_code__update(p_target, ERROR_TARGET_UNALIGNED_ACCESS);
		return error_code__clear(p_target);
	}
	if (breakpoint->type != BKPT_SOFT) {
		error_code__update(p_target, ERROR_TARGET_RESOURCE_NOT_AVAILABLE);
		return error_code__clear(p_target);
	}

	assert(p_target);
	update_status(p_target);
	if (error_code__get(p_target) != ERROR_OK) {
		return error_code__clear(p_target);
	}
	if (p_target->state != TARGET_HALTED) {
		error_code__update(p_target, ERROR_TARGET_NOT_HALTED);
		return error_code__clear(p_target);
	}

	return target_write_buffer(p_target, breakpoint->address, breakpoint->length, breakpoint->orig_instr);
}

/// gdb_server expects valid reg values and will use set method for updating reg values
static int
this_get_gdb_reg_list(target* const restrict p_target, reg **reg_list[], int* const restrict reg_list_size, enum target_register_class const reg_class)
{
	This_Arch *arch_info = p_target->arch_info;
	assert(arch_info);

	size_t const num_regs = reg_class == REG_CLASS_ALL ? TOTAL_NUMBER_OF_REGS : REG_PC_NUMBER + 1;
	reg** p_reg_array = calloc(num_regs, sizeof(reg*));
	reg *a_reg_list = p_target->reg_cache->reg_list;
	for (size_t i = 0; i < num_regs; ++i) {
		p_reg_array[i] = &a_reg_list[i];
	}
	*reg_list_size = num_regs;
	*reg_list = p_reg_array;
	return error_code__clear(p_target);
}

target_type syntacore_riscv32i_target =
{
	.name = "syntacore_riscv32i",

	.poll = this_poll,
	.arch_state = this_arch_state,
	.target_request_data = NULL,

	.halt = this_halt,
	.resume = this_resume,
	.step = this_step,

	.assert_reset = this_assert_reset,
	.deassert_reset = this_deassert_reset,
	.soft_reset_halt = this_soft_reset_halt,

	.get_gdb_reg_list = this_get_gdb_reg_list,

	.read_memory = this_read_memory,
	.write_memory = this_write_memory,

	.read_buffer = NULL,
	.write_buffer = NULL,

	.checksum_memory = NULL,
	.blank_check_memory = NULL,

	.add_breakpoint = this_add_breakpoint,
	.add_context_breakpoint = NULL,
	.add_hybrid_breakpoint = NULL,

	.remove_breakpoint = this_remove_breakpoint,

	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.hit_watchpoint = NULL,

	.run_algorithm = NULL,
	.start_algorithm = NULL,
	.wait_algorithm = NULL,

	.commands = NULL,

	.target_create = this_target_create,
	.target_jim_configure = NULL,
	.target_jim_commands = NULL,

	.examine = this_examine,

	.init_target = this_init_target,
	.deinit_target = this_deinit_target,

	.virt2phys = NULL,
	.read_phys_memory = NULL,
	.write_phys_memory = NULL,

	.mmu = NULL,
	.check_reset = NULL,
	.get_gdb_fileio_info = NULL,
	.gdb_fileio_end = NULL,
	.profiling = NULL,
};

