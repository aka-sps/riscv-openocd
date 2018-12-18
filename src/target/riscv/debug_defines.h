#ifndef TARGET_RISCV_DEBUG_DEFINES_H_
#define TARGET_RISCV_DEBUG_DEFINES_H_

#include <stdint.h>

#if 0
/** @name Identifies the release version of this part.*/
/**@{*/
#define DTM_IDCODE                          (0x01)
#define DTM_IDCODE_VERSION_OFFSET           (28)
#if 0
#define DTM_IDCODE_VERSION_LENGTH           (4)
#endif
#define DTM_IDCODE_VERSION                  (UINT32_C(0xF) << DTM_IDCODE_VERSION_OFFSET)
/**@}*/

/** @name Identifies the designer's part number of this part. */
/**@{*/
#define DTM_IDCODE_PARTNUMBER_OFFSET        (12)
#if 0
#define DTM_IDCODE_PARTNUMBER_LENGTH        (16)
#endif
#define DTM_IDCODE_PARTNUMBER               (UINT32_C(0xFFFF) << DTM_IDCODE_PARTNUMBER_OFFSET)
/**@}*/

/** @name DTM_IDCODE

Identifies the designer/manufacturer of this part.

Bits 6:0 must be bits 6:0 of the designer/manufacturer's Identification Code as
assigned by JEDEC Standard JEP106.
Bits 10:7 contain the modulo-16 count of the number of continuation characters (0x7f)
in that same Identification Code.
*/
/**@{*/
#define DTM_IDCODE_MANUFID_OFFSET           (1)
#define DTM_IDCODE_MANUFID_LENGTH           (11)
#define DTM_IDCODE_MANUFID                  (UINT32_C(0x7FF) << DTM_IDCODE_MANUFID_OFFSET)
#define DTM_IDCODE_1_OFFSET                 (0)
#define DTM_IDCODE_1_LENGTH                 (1)
#define DTM_IDCODE_1                        (UINT32_C(0x1) << DTM_IDCODE_1_OFFSET)
/**@}*/
#endif

#define DTM_DTMCS                           (0x10)

#if 0
/** @name DTM_DTMCS_DMIHARDRESET

Writing 1 to this bit does a hard reset of the DTM,
causing the DTM to forget about any outstanding DMI transactions.
In general this should only be used when the Debugger has
reason to expect that the outstanding DMI transaction will never
complete (e.g. a reset condition caused an in flight DMI transaction to
be canceled).
*/
/**@{*/
#define DTM_DTMCS_DMIHARDRESET_OFFSET       (17)
#if 0
#define DTM_DTMCS_DMIHARDRESET_LENGTH       (1)
#endif
#define DTM_DTMCS_DMIHARDRESET              (UINT32_C(0x1) << DTM_DTMCS_DMIHARDRESET_OFFSET)
/**@}*/

#endif

/** @name DTM_DTMCS_DMIRESET

Writing 1 to this bit clears the sticky error state
and allows the DTM to retry or complete the previous
transaction.
*/
/**@{*/
#define DTM_DTMCS_DMIRESET_OFFSET           (16)
#if 0
#define DTM_DTMCS_DMIRESET_LENGTH           (1)
#endif
#define DTM_DTMCS_DMIRESET                  (UINT32_C(0x1) << DTM_DTMCS_DMIRESET_OFFSET)
/**@}*/

/** @name DTM_DTMCS_IDLE

This is a hint to the debugger of the minimum number of
cycles a debugger should spend in
Run-Test/Idle after every DMI scan to avoid a `busy'
return code (@c dmistat of 3). A debugger must still
check @c dmistat when necessary.

0: It is not necessary to enter Run-Test/Idle at all.

1: Enter Run-Test/Idle and leave it immediately.

2: Enter Run-Test/Idle and stay there for 1 cycle before leaving.

And so on.
*/
/**@{*/
#define DTM_DTMCS_IDLE_OFFSET               (12)
#if 0
#define DTM_DTMCS_IDLE_LENGTH               (3)
#endif
#define DTM_DTMCS_IDLE                      (UINT32_C(0x7) << DTM_DTMCS_IDLE_OFFSET)
/**@}*/

/** @name DTM_DTMCS_DMISTAT

0: No error.

1: Reserved. Interpret the same as 2.

2: An operation failed (resulted in @c op of 2).

3: An operation was attempted while a DMI access was still in
progress (resulted in @c op of 3).
*/
/**@{*/
#define DTM_DTMCS_DMISTAT_OFFSET            (10)
#if 0
#define DTM_DTMCS_DMISTAT_LENGTH            (2)
#endif
#define DTM_DTMCS_DMISTAT                   (UINT32_C(0x3) << DTM_DTMCS_DMISTAT_OFFSET)
/**@}*/

/** @name DTM_DTMCS_ABITS
	The size of @c address in @c dmi.
*/
/**@{*/
#define DTM_DTMCS_ABITS_OFFSET              (4)
#if 0
#define DTM_DTMCS_ABITS_LENGTH              (6)
#endif
#define DTM_DTMCS_ABITS                     (UINT32_C(0x3F) << DTM_DTMCS_ABITS_OFFSET)
/**@}*/

/** @name DTM_DTMCS_VERSION

0: Version described in spec version 0.11.

1: Version described in spec version 0.13 (and later?), which
reduces the DMI data width to 32 bits.

15: Version not described in any available version of this spec.
*/
/**@{*/
#define DTM_DTMCS_VERSION_OFFSET            (0)
#if 0
#define DTM_DTMCS_VERSION_LENGTH            (4)
#endif
#define DTM_DTMCS_VERSION                   (UINT32_C(0xF) << DTM_DTMCS_VERSION_OFFSET)
/**@}*/

#define DTM_DMI                             (0x11)

/** @name DTM_DMI_ADDRESS

	Address used for DMI access. In Update-DR this value is used
	to access the DM over the DMI.
*/
/**@{*/
#define DTM_DMI_ADDRESS_OFFSET              (34)
#if 0
#define DTM_DMI_ADDRESS_LENGTH              (abits)
#define DTM_DMI_ADDRESS                     (((1L << (abits)) - 1) << DTM_DMI_ADDRESS_OFFSET)
#endif
/**@}*/

/** @name DTM_DMI_DATA

	The data to send to the DM over the DMI during Update-DR, and
	the data returned from the DM as a result of the previous operation.
*/
/**@{*/
#define DTM_DMI_DATA_OFFSET                 (2)
#define DTM_DMI_DATA_LENGTH                 (32)
#define DTM_DMI_DATA                        ((uint64_t)(UINT32_MAX) << DTM_DMI_DATA_OFFSET)
/**@}*/

/** @name DTM_DMI_OP

	When the debugger writes this field, it has the following meaning:

	0: Ignore @c data and @c address. (nop)

	Don't send anything over the DMI during Update-DR.
	This operation should never result in a busy or error response.
	The address and data reported in the following Capture-DR
	are undefined.

	1: Read from @c address. (read)

	2: Write @c data to @c address. (write)

	3: Reserved.

	When the debugger reads this field, it means the following:

	0: The previous operation completed successfully.

	1: Reserved.

	2: A previous operation failed.  The data scanned into @c dmi in
	this access will be ignored.  This status is sticky and can be
	cleared by writing @c dmireset in @c dtmcs.

	This indicates that the DM itself responded with an error.
	Note: there are no specified cases in which the DM would
	respond with an error, and DMI is not required to support
	returning errors.

	3: An operation was attempted while a DMI request is still in
	progress. The data scanned into @c dmi in this access will be
	ignored. This status is sticky and can be cleared by writing
	@c dmireset in @c dtmcs. If a debugger sees this status, it
	needs to give the target more TCK edges between Update-DR and
	Capture-DR. The simplest way to do that is to add extra transitions
	in Run-Test/Idle.

	(The DTM, DM, and/or component may be in different clock domains,
	so synchronization may be required. Some relatively fixed number of
	TCK ticks may be needed for the request to reach the DM, complete,
	and for the response to be synchronized back into the TCK domain.)
*/
/**@{*/
#define DTM_DMI_OP_OFFSET                   (0)
#define DTM_DMI_OP_LENGTH                   (2)
#define DTM_DMI_OP                          (UINT64_C(0x3) << DTM_DMI_OP_OFFSET)
/**@}*/

#if 0
/** @name CSR_DCSR_XDEBUGVER

	0: There is no external debug support.

	4: External debug support exists as it is described in this document.

	15: There is external debug support, but it does not conform to any
	available version of this spec.
*/
/**@{*/
#define CSR_DCSR_XDEBUGVER_OFFSET           (28)
#if 0
#define CSR_DCSR_XDEBUGVER_LENGTH           (4)
#endif
#define CSR_DCSR_XDEBUGVER                  (UINT32_C(0xF) << CSR_DCSR_XDEBUGVER_OFFSET)
/**@}*/
#endif

/** @name CSR_DCSR_EBREAKM

	When 1, @c ebreak instructions in Machine Mode enter Debug Mode.
*/
/**@{*/
#define CSR_DCSR_EBREAKM_OFFSET             (15)
#if 0
#define CSR_DCSR_EBREAKM_LENGTH             (1)
#endif
#define CSR_DCSR_EBREAKM                    (0x1u << CSR_DCSR_EBREAKM_OFFSET)
/**@}*/

/** @name CSR_DCSR_EBREAKS
	When 1, @c ebreak instructions in Supervisor Mode enter Debug Mode.
*/
/**@{*/
#define CSR_DCSR_EBREAKS_OFFSET             (13)
#if 0
#define CSR_DCSR_EBREAKS_LENGTH             (1)
#endif
#define CSR_DCSR_EBREAKS                    (UINT32_C(0x1) << CSR_DCSR_EBREAKS_OFFSET)
/**@}*/

/** @name CSR_DCSR_EBREAKU
	When 1, @ ebreak instructions in User/Application Mode enter Debug Mode.
*/
/**@{*/
#define CSR_DCSR_EBREAKU_OFFSET             (12)
#if 0
#define CSR_DCSR_EBREAKU_LENGTH             (1)
#endif
#define CSR_DCSR_EBREAKU                    (UINT32_C(0x1) << CSR_DCSR_EBREAKU_OFFSET)
/**@}*/

#if 0
/** @name CSR_DCSR_STEPIE
	0: Interrupts are disabled during single stepping.

	1: Interrupts are enabled during single stepping.

	Implementations may hard wire this bit to 0.
	The debugger must read back the value it
	writes to check whether the feature is supported. If not
	supported, interrupt behavior can be emulated by the debugger.
*/
/**@{*/
#define CSR_DCSR_STEPIE_OFFSET              (11)
#if 0
#define CSR_DCSR_STEPIE_LENGTH              (1)
#endif
#define CSR_DCSR_STEPIE                     (0x1u << CSR_DCSR_STEPIE_OFFSET)
/**@}*/

/** @name CSR_DCSR_STOPCOUNT

	0: Increment counters as usual.

	1: Don't increment any counters while in Debug Mode or on
	@c ebreak instructions that cause entry into Debug Mode.  These
	counters include the @c  cycle and @c  instret CSRs. This is
	preferred for most debugging scenarios.

	An implementation may choose not to support writing to this bit.
	The debugger must read back the value it writes to check whether
	the feature is supported.
*/
/**@{*/
#define CSR_DCSR_STOPCOUNT_OFFSET           (10)
#if 0
#define CSR_DCSR_STOPCOUNT_LENGTH           (1)
#endif
#define CSR_DCSR_STOPCOUNT                  (0x1u << CSR_DCSR_STOPCOUNT_OFFSET)
/**@}*/

/** @name CSR_DCSR_STOPTIME

	0: Increment timers as usual.

	1: Don't increment any hart-local timers while in Debug Mode.

	An implementation may choose not to support writing to this bit.
	The debugger must read back the value it writes to check whether
	the feature is supported.
*/
/**@{*/
#define CSR_DCSR_STOPTIME_OFFSET            (9)
#if 0
#define CSR_DCSR_STOPTIME_LENGTH            (1)
#endif
#define CSR_DCSR_STOPTIME                   (UINT32_C(0x1) << CSR_DCSR_STOPTIME_OFFSET)
/**@}*/
#endif

/** @name CSR_DCSR_CAUSE
	@brief Explains why Debug Mode was entered.

	When there are multiple reasons to enter Debug Mode in a single
	cycle, hardware should set @c cause to the cause with the highest
	priority.

	1: An @c ebreak instruction was executed. (priority 3)

	2: The Trigger Module caused a breakpoint exception. (priority 4)

	3: The debugger requested entry to Debug Mode. (priority 2)

	4: The hart single stepped because @c step was set. (priority 1)

	Other values are reserved for future use.
*/
/**@{*/
#define CSR_DCSR_CAUSE_OFFSET               (6)
#if 0
#define CSR_DCSR_CAUSE_LENGTH               (3)
#endif
#define CSR_DCSR_CAUSE                      (UINT32_C(0x7) << CSR_DCSR_CAUSE_OFFSET)
/**@}*/

#if 0
/** @name CSR_DCSR_MPRVEN
	When 1, @c mprv in @c mstatus takes effect during debug mode.
	When 0, it is ignored during debug mode.
	Implementing this bit is optional.
	If not implemented it should be tied to 0.
*/
/**@{*/
#define CSR_DCSR_MPRVEN_OFFSET              (4)
#if 0
#define CSR_DCSR_MPRVEN_LENGTH              (1)
#endif
#define CSR_DCSR_MPRVEN                     (UINT32_C(0x1) << CSR_DCSR_MPRVEN_OFFSET)
/**@}*/

/** @name CSR_DCSR_NMIP

When set, there is a Non-Maskable-Interrupt (NMI) pending for the hart.

Since an NMI can indicate a hardware error condition,
reliable debugging may no longer be possible once this bit becomes set.
This is implementation-dependent.
*/
/**@{*/
#define CSR_DCSR_NMIP_OFFSET                (3)
#if 0
#define CSR_DCSR_NMIP_LENGTH                (1)
#endif
#define CSR_DCSR_NMIP                       (0x1u << CSR_DCSR_NMIP_OFFSET)
/**@}*/
#endif

/** @name CSR_DCSR_STEP

	When set and not in Debug Mode, the hart will only execute a single
	instruction and then enter Debug Mode.
	If the instruction does not complete due to an exception,
	the hart will immediately enter Debug Mode before executing
	the trap handler, with appropriate exception registers set.
*/
/**@{*/
#define CSR_DCSR_STEP_OFFSET                (2)
#if 0
#define CSR_DCSR_STEP_LENGTH                (1)
#endif
#define CSR_DCSR_STEP                       (0x1u << CSR_DCSR_STEP_OFFSET)
/**@}*/

/** @name CSR_DCSR_PRV
	Contains the privilege level the hart was operating in when Debug
	Mode was entered. The encoding is described in Table \ref{tab:privlevel}.
	A debugger can change this value to change
	the hart's privilege level when exiting Debug Mode.

	Not all privilege levels are supported on all harts. If the
	encoding written is not supported or the debugger is not allowed to
	change to it, the hart may change to any supported privilege level.
*/
/**@{*/
#define CSR_DCSR_PRV_OFFSET                 (0)
#if 0
#define CSR_DCSR_PRV_LENGTH                 (2)
#endif
#define CSR_DCSR_PRV                        (UINT32_C(0x3) << CSR_DCSR_PRV_OFFSET)
/**@}*/

#if 0
/** @name CSR_DPC_DPC */
/**@{*/
#define CSR_DPC_DPC_OFFSET                  (0)
#define CSR_DPC_DPC_LENGTH                  (MXLEN)
#define CSR_DPC_DPC                         (((1L << (MXLEN)) - 1) << CSR_DPC_DPC_OFFSET)
/**@}*/

#define CSR_DSCRATCH0                       (0x7B2)
#define CSR_DSCRATCH1                       (0x7B3)

/** @name CSR_TSELECT_INDEX */
/**@{*/
#define CSR_TSELECT_INDEX_OFFSET            (0)
#define CSR_TSELECT_INDEX_LENGTH            (MXLEN)
#define CSR_TSELECT_INDEX                   (((1L << (MXLEN)) - 1) << CSR_TSELECT_INDEX_OFFSET)
/**@}*/
#endif
/**@}*/

#if 0
/** @name CSR_TDATA1_TYPE
	0: There is no trigger at this @c tselect.

	1: The trigger is a legacy SiFive address match trigger. These
	should not be implemented and aren't further documented here.

	2: The trigger is an address/data match trigger. The remaining bits
	in this register act as described in @c mcontrol.

	3: The trigger is an instruction count trigger. The remaining bits
	in this register act as described in @c icount.

	4: The trigger is an interrupt trigger. The remaining bits
	in this register act as described in @c itrigger.

	5: The trigger is an exception trigger. The remaining bits
	in this register act as described in @c etrigger.

	15: This trigger exists (so enumeration shouldn't terminate), but
	is not currently available.

	Other values are reserved for future use.

	When this field is written to an unsupported value, it takes on its
	reset value instead. The reset value is any one of the types
	supported by the trigger selected by @c tselect.
*/
/**@{*/
#define CSR_TDATA1_TYPE_OFFSET              ((MXLEN)-4)
#define CSR_TDATA1_TYPE_LENGTH              (4)
#define CSR_TDATA1_TYPE                     (UINT64_C(0xF) << CSR_TDATA1_TYPE_OFFSET)
/**@}*/

/** @name Trigger-specific data.*/
/**@{*/

/** @name CSR_TDATA1_DMODE
	0: Both Debug and M Mode can write the @c tdata registers at the
	selected @c tselect.

	1: Only Debug Mode can write the @c tdata registers at the
	selected @c tselect.  Writes from other modes are ignored.

	This bit is only writable from Debug Mode.
*/
/**@{*/
#define CSR_TDATA1_DMODE_OFFSET             ((MXLEN) - 5)
#define CSR_TDATA1_DMODE_LENGTH             (1)
#define CSR_TDATA1_DMODE                    (UINT64_C(0x1) << CSR_TDATA1_DMODE_OFFSET)
/**@}*/

/** @name CSR_TDATA1 */
/**@{*/
#define CSR_TDATA1_DATA_OFFSET              (0)
#define CSR_TDATA1_DATA_LENGTH              ((MXLEN) - 5)
#define CSR_TDATA1_DATA                     (((1L << (MXLEN) - 5) - 1) << CSR_TDATA1_DATA_OFFSET)
/**@}*/

/** @name CSR_TDATA2_DATA */
/**@{*/
#define CSR_TDATA2_DATA_OFFSET              (0)
#define CSR_TDATA2_DATA_LENGTH              (MXLEN)
#define CSR_TDATA2_DATA                     (((1L << (MXLEN)) - 1) << CSR_TDATA2_DATA_OFFSET)
/**@}*/

/** @name CSR_TDATA3_DATA */
/**@{*/
#define CSR_TDATA3_DATA_OFFSET              (0)
#define CSR_TDATA3_DATA_LENGTH              (MXLEN)
#define CSR_TDATA3_DATA                     (((1L << (MXLEN)) - 1) << CSR_TDATA3_DATA_OFFSET)
/**@}*/

#define CSR_TINFO                           (0x7A4)
/**@}*/

/**
	One bit for each possible @c type enumerated in @c tdataone.
	Bit N corresponds to type N.
	If the bit is set, then that type is supported by the currently selected trigger.

	If the currently selected trigger doesn't exist, this field contains 1.

	If @c type is not writable, this register may be unimplemented,
	in which case reading it causes an illegal instruction exception.
	In this case the debugger can read the only supported type from
	@c tdataone.
*/
/**@{*/
#define CSR_TINFO_INFO_OFFSET               (0)
#define CSR_TINFO_INFO_LENGTH               (16)
#define CSR_TINFO_INFO                      (UINT64_C(0xFFFF) << CSR_TINFO_INFO_OFFSET)
#define CSR_MCONTROL                        (0x7A1)
#define CSR_MCONTROL_TYPE_OFFSET            ((MXLEN) - 4)
#define CSR_MCONTROL_TYPE_LENGTH            (4)
#define CSR_MCONTROL_TYPE                   (UINT64_C(0xF) << CSR_MCONTROL_TYPE_OFFSET)
#define CSR_MCONTROL_DMODE_OFFSET           ((MXLEN) - 5)
#define CSR_MOCNTROL_DMODE_LENGTH           (1)
#define CSR_MCONTROL_DMODE                  (UINT64_C(0x1) << CSR_MCONTROL_DMODE_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_MASKMAX
	Specifies the largest naturally aligned powers-of-two (NAPOT) range
	supported by the hardware when @c match is 1. The value is the
	logarithm base 2 of the number of bytes in that range.
	A value of 0 indicates that only exact value matches are supported (one byte range).
	A value of 63 corresponds to the maximum NAPOT range,
	which is \f$2^{63}\f$ bytes in size.
*/
/**@{*/
#define CSR_MCONTROL_MASKMAX_OFFSET         ((MXLEN) - 11)
#define CSR_MCONTROL_MASKMAX_LENGTH         (6)
#define CSR_MCONTROL_MASKMAX                (UINT64_C(0x3F) << CSR_MCONTROL_MASKMAX_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_HIT
	If this optional bit is implemented, the hardware sets it when this
	trigger matches. The trigger's user can set or clear it at any
	time. The trigger's user can use this bit to determine which
	trigger(s) matched.  If the bit is not implemented, it is always 0
	and writing it has no effect.
*/
/**@{*/
#define CSR_MCONTROL_HIT_OFFSET             (20)
#if 0
#define CSR_MCONTROL_HIT_LENGTH             (1)
#endif
#define CSR_MCONTROL_HIT                    (UINT64_C(0x1) << CSR_MCONTROL_HIT_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_SELECT
	0: Perform a match on the virtual address.

	1: Perform a match on the data value loaded/stored, or the
	instruction executed.
*/
/**@{*/
#define CSR_MCONTROL_SELECT_OFFSET          (19)
#if 0
#define CSR_MCONTROL_SELECT_LENGTH          (1)
#endif
#define CSR_MCONTROL_SELECT                 (UINT64_C(0x1) << CSR_MCONTROL_SELECT_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_TIMING
	0: The action for this trigger will be taken just before the
	instruction that triggered it is executed, but after all preceding
	instructions are are committed.

	1: The action for this trigger will be taken after the instruction
	that triggered it is executed. It should be taken before the next
	instruction is executed, but it is better to implement triggers and
	not implement that suggestion than to not implement them at all.

	Most hardware will only implement one timing or the other, possibly
	dependent on @c select, @c execute, @c load, and @c store.
	This bit primarily exists for the hardware to communicate to the debugger
	what will happen. Hardware may implement the bit fully writable, in
	which case the debugger has a little more control.

	Data load triggers with @c timing of 0 will result in the same load
	happening again when the debugger lets the hart run. For data load
	triggers, debuggers must first attempt to set the breakpoint with
	@c timing of 1.

	A chain of triggers that don't all have the same @c timing value
	will never fire (unless consecutive instructions match the
	appropriate triggers).
*/
/**@{*/
#define CSR_MCONTROL_TIMING_OFFSET          (18)
#if 0
#define CSR_MCONTROL_TIMING_LENGTH          (1)
#endif
#define CSR_MCONTROL_TIMING                 (UINT64_C(0x1) << CSR_MCONTROL_TIMING_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_ACTION
The action to take when the trigger fires. The values are explained
in Table \ref{tab:action}.
*/
/**@{*/
#define CSR_MCONTROL_ACTION_OFFSET          (12)
#if 0
#define CSR_MCONTROL_ACTION_LENGTH          (6)
#endif
#define CSR_MCONTROL_ACTION                 (UINT64_C(0x3F) << CSR_MCONTROL_ACTION_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_CHAIN

0: When this trigger matches, the configured action is taken.

1: While this trigger does not match, it prevents the trigger with
the next index from matching.

Because @c chain affects the next trigger, hardware must zero it in
writes to @c mcontrol that set @c dmode to 0 if the next trigger has
@c dmode of 1.

In addition hardware should ignore writes to @c mcontrol that set
@c dmode to 1 if the previous trigger has both @c dmode of 0 and
@c chain of 1. Debuggers must avoid the latter case by checking
@c chain on the previous trigger if they're writing @c mcontrol.

Implementations that wish to limit the maximum length of a trigger
chain (e.g. to meet timing requirements) may do so by zeroing
@c chain in writes to @c mcontrol that would make the chain too long.
*/
/**@{*/
#define CSR_MCONTROL_CHAIN_OFFSET           (11)
#if 0
#define CSR_MCONTROL_CHAIN_LENGTH           (1)
#endif
#define CSR_MCONTROL_CHAIN                  (UINT64_C(0x1) << CSR_MCONTROL_CHAIN_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_MATCH

0: Matches when the value equals @c tdatatwo.

1: Matches when the top M bits of the value match the top M bits of
@c tdatatwo. M is MXLEN-1 minus the index of the least-significant
bit containing 0 in @c tdatatwo.

2: Matches when the value is greater than (unsigned) or equal to
@c tdatatwo.

3: Matches when the value is less than (unsigned) @c tdatatwo.

4: Matches when the lower half of the value equals the lower half
of @c tdatatwo after the lower half of the value is ANDed with the
upper half of @c tdatatwo.

5: Matches when the upper half of the value equals the lower half
of @c tdatatwo after the upper half of the value is ANDed with the
upper half of @c tdatatwo.

Other values are reserved for future use.
*/
/**@{*/
#define CSR_MCONTROL_MATCH_OFFSET           (7)
#define CSR_MCONTROL_MATCH_LENGTH           (4)
#define CSR_MCONTROL_MATCH                  (UINT64_C(0xF) << CSR_MCONTROL_MATCH_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_M

When set, enable this trigger in M mode.
 */
/**@{*/
#define CSR_MCONTROL_M_OFFSET               (6)
#define CSR_MCONTROL_M_LENGTH               (1)
#define CSR_MCONTROL_M                      (UINT64_C(0x1) << CSR_MCONTROL_M_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_S

When set, enable this trigger in S mode.
*/
/**@{*/
#define CSR_MCONTROL_S_OFFSET               (4)
#define CSR_MCONTROL_S_LENGTH               (1)
#define CSR_MCONTROL_S                      (UINT64_C(0x1) << CSR_MCONTROL_S_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_U

When set, enable this trigger in U mode.
*/
/**@{*/
#define CSR_MCONTROL_U_OFFSET               (3)
#define CSR_MCONTROL_U_LENGTH               (1)
#define CSR_MCONTROL_U                      (UINT64_C(0x1) << CSR_MCONTROL_U_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_EXECUTE

When set, the trigger fires on the virtual address or opcode of an
instruction that is executed.
*/
/**@{*/
#define CSR_MCONTROL_EXECUTE_OFFSET         (2)
#define CSR_MCONTROL_EXECUTE_LENGTH         (1)
#define CSR_MCONTROL_EXECUTE                (UINT64_C(0x1) << CSR_MCONTROL_EXECUTE_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_STORE

When set, the trigger fires on the virtual address or data of a store.
*/
/**@{*/
#define CSR_MCONTROL_STORE_OFFSET           (1)
#define CSR_MCONTROL_STORE_LENGTH           (1)
#define CSR_MCONTROL_STORE                  (UINT64_C(0x1) << CSR_MCONTROL_STORE_OFFSET)
/**@}*/

/** @name CSR_MCONTROL_LOAD

When set, the trigger fires on the virtual address or data of a load.
*/
/**@{*/
#define CSR_MCONTROL_LOAD_OFFSET            (0)
#define CSR_MCONTROL_LOAD_LENGTH            (1)
#define CSR_MCONTROL_LOAD                   (UINT64_C(0x1) << CSR_MCONTROL_LOAD_OFFSET)
/**@}*/

/** @name CSR_ICOUNT

When set, the trigger fires on the virtual address or data of a load.
*/
/**@{*/
#define CSR_ICOUNT                          (0x7A1)
#define CSR_ICOUNT_TYPE_OFFSET              ((MXLEN) - 4)
#define CSR_ICOUNT_TYPE_LENGTH              (4)
#define CSR_ICOUNT_TYPE                     (UINT64_C(0xF) << CSR_ICOUNT_TYPE_OFFSET)
#define CSR_ICOUNT_DMODE_OFFSET             ((MXLEN) - 5)
#define CSR_ICOUNT_DMODE_LENGTH             (1)
#define CSR_ICOUNT_DMODE                    (UINT64_C(0x1) << CSR_ICOUNT_DMODE_OFFSET)
/**@}*/

/** @name CSR_ICOUNT_HIT

If this optional bit is implemented, the hardware sets it when this
trigger matches. The trigger's user can set or clear it at any
time. The trigger's user can use this bit to determine which
trigger(s) matched.  If the bit is not implemented, it is always 0
and writing it has no effect.
*/
/**@{*/
#define CSR_ICOUNT_HIT_OFFSET               (24)
#if 0
#define CSR_ICOUNT_HIT_LENGTH               (1)
#endif
#define CSR_ICOUNT_HIT                      (UINT64_C(0x1) << CSR_ICOUNT_HIT_OFFSET)
/**@}*/

/** @name CSR_ICOUNT_COUNT

When count is decremented to 0, the trigger fires. Instead of
changing @c count from 1 to 0, it is also acceptable for hardware to
clear @c m, @c s, and @c u. This allows @c count to be hard-wired
to 1 if this register just exists for single step.
*/
/**@{*/
#define CSR_ICOUNT_COUNT_OFFSET             (10)
#define CSR_ICOUNT_COUNT_LENGTH             (14)
#define CSR_ICOUNT_COUNT                    (UINT64_C(0x3FFF) << CSR_ICOUNT_COUNT_OFFSET)
/**@}*/

/** @name CSR_ICOUNT_M

When set, every instruction completed or exception taken in M mode decrements @c count by 1.
*/
/**@{*/
#define CSR_ICOUNT_M_OFFSET                 (9)
#define CSR_ICOUNT_M_LENGTH                 (1)
#define CSR_ICOUNT_M                        (UINT64_C(0x1) << CSR_ICOUNT_M_OFFSET)
/**@}*/

/** @name CSR_ICOUNT_S

When set, every instruction completed or exception taken in S mode decrements @c count by 1.
*/
/**@{*/
#define CSR_ICOUNT_S_OFFSET                 (7)
#define CSR_ICOUNT_S_LENGTH                 (1)
#define CSR_ICOUNT_S                        (UINT64_C(0x1) << CSR_ICOUNT_S_OFFSET)
/**@}*/

/** @ name CSR_ICOUNT_U

When set, every instruction completed or exception taken in U mode decrements @c count by 1.
*/
/**@{*/
#define CSR_ICOUNT_U_OFFSET                 (6)
#define CSR_ICOUNT_U_LENGTH                 (1)
#define CSR_ICOUNT_U                        (UINT64_C(0x1) << CSR_ICOUNT_U_OFFSET)
/**@}*/

/** @name CSR_ICOUNT_ACTION
The action to take when the trigger fires.

The values are explained in Table~\ref{tab:action}.
*/
/**@{*/
#define CSR_ICOUNT_ACTION_OFFSET            (0)
#define CSR_ICOUNT_ACTION_LENGTH            (6)
#define CSR_ICOUNT_ACTION                   (UINT64_C(0x3F) << CSR_ICOUNT_ACTION_OFFSET)
#define CSR_ITRIGGER                        (0x7A1)
#define CSR_ITRIGGER_TYPE_OFFSET            ((MXLEN) - 4)
#define CSR_ITRIGGER_TYPE_LENGTH            (4)
#define CSR_ITRIGGER_TYPE                   (UINT64_C(0xF) << CSR_ITRIGGER_TYPE_OFFSET)
#define CSR_ITRIGGER_DMODE_OFFSET           ((MXLEN) - 5)
#define CSR_ITRIGGER_DMODE_LENGTH           (1)
#define CSR_ITRIGGER_DMODE                  (UINT64_C(0x1) << CSR_ITRIGGER_DMODE_OFFSET)
/**@}*/

/** @name CSR_ITRIGGER_HIT

If this optional bit is implemented, the hardware sets it when this
trigger matches. The trigger's user can set or clear it at any
time. The trigger's user can use this bit to determine which
trigger(s) matched.  If the bit is not implemented, it is always 0
and writing it has no effect.
*/
/**@{*/
#define CSR_ITRIGGER_HIT_OFFSET             ((MXLEN) - 6)
#define CSR_ITRIGGER_HIT_LENGTH             (1)
#define CSR_ITRIGGER_HIT                    (UINT64_C(0x1) << CSR_ITRIGGER_HIT_OFFSET)
/**@}*/

/** @name CSR_ITRIGGER_M

When set, enable this trigger for interrupts that are taken from M mode.
*/
/**@{*/
#define CSR_ITRIGGER_M_OFFSET               (9)
#define CSR_ITRIGGER_M_LENGTH               (1)
#define CSR_ITRIGGER_M                      (UINT64_C(0x1) << CSR_ITRIGGER_M_OFFSET)
/**@}*/

/** @name CSR_ITRIGGER_S

When set, enable this trigger for interrupts that are taken from S mode.
*/
/**@{*/
#define CSR_ITRIGGER_S_OFFSET               (7)
#define CSR_ITRIGGER_S_LENGTH               (1)
#define CSR_ITRIGGER_S                      (UINT64_C(0x1) << CSR_ITRIGGER_S_OFFSET)
/**@}*/

/** @name CSR_ITRIGGER_U
When set, enable this trigger for interrupts that are taken from U mode.
*/
/**@{*/
#define CSR_ITRIGGER_U_OFFSET               (6)
#define CSR_ITRIGGER_U_LENGTH               (1)
#define CSR_ITRIGGER_U                      (UINT64_C(0x1) << CSR_ITRIGGER_U_OFFSET)
/**@}*/

/** @name CSR_ETRIGGER

The action to take when the trigger fires.
The values are explained in Table~\ref{tab:action}.
*/
/**@{*/
#define CSR_ITRIGGER_ACTION_OFFSET          (0)
#define CSR_ITRIGGER_ACTION_LENGTH          (6)
#define CSR_ITRIGGER_ACTION                 (UINT64_C(0x3F) << CSR_ITRIGGER_ACTION_OFFSET)
#define CSR_ETRIGGER                        (0x7A1)
#define CSR_ETRIGGER_TYPE_OFFSET            ((MXLEN) - 4)
#define CSR_ETRIGGER_TYPE_LENGTH            (4)
#define CSR_ETRIGGER_TYPE                   (UINT64_C(0xf) << CSR_ETRIGGER_TYPE_OFFSET)
#define CSR_ETRIGGER_DMODE_OFFSET           ((MXLEN) - 5)
#define CSR_ETRIGGER_DMODE_LENGTH           (1)
#define CSR_ETRIGGER_DMODE                  (UINT64_C(0x1) << CSR_ETRIGGER_DMODE_OFFSET)
/**@}*/

/** @name CSR_ETRIGGER_HIT

If this optional bit is implemented, the hardware sets it when this
trigger matches. The trigger's user can set or clear it at any
time. The trigger's user can use this bit to determine which
trigger(s) matched.  If the bit is not implemented, it is always 0
and writing it has no effect.
*/
/**@{*/
#define CSR_ETRIGGER_HIT_OFFSET             ((MXLEN) - 6)
#define CSR_ETRIGGER_HIT_LENGTH             (1)
#define CSR_ETRIGGER_HIT                    (UINT64_C(0x1) << CSR_ETRIGGER_HIT_OFFSET)
/**@}*/

/** @name CSR_ETRIGGER_mode */
/**@{*/
/** @name CSR_ETRIGGER_M

When set, enable this trigger for exceptions that are taken from M mode.
*/
/**@{*/
#define CSR_ETRIGGER_M_OFFSET               (9)
#define CSR_ETRIGGER_M_LENGTH               (1)
#define CSR_ETRIGGER_M                      (UINT64_C(0x1) << CSR_ETRIGGER_M_OFFSET)
/**@}*/

/** @name CSR_ETRIGGER_S

When set, enable this trigger for exceptions that are taken from S mode.
*/
/**@{*/
#define CSR_ETRIGGER_S_OFFSET               (7)
#define CSR_ETRIGGER_S_LENGTH               (1)
#define CSR_ETRIGGER_S                      (UINT64_C(0x1) << CSR_ETRIGGER_S_OFFSET)
/**@}*/

/** @name CSR_ETRIGGER_U

When set, enable this trigger for exceptions that are taken from U mode.
 */
 /**@{*/
#define CSR_ETRIGGER_U_OFFSET               (6)
#define CSR_ETRIGGER_U_LENGTH               (1)
#define CSR_ETRIGGER_U                      (UINT64_C(0x1) << CSR_ETRIGGER_U_OFFSET)
/**@}*/

/** @name CSR_ETRIGGER_ACTION

	@bug TeX markup

The action to take when the trigger fires. The values are explained
in Table tab:action.
*/
/**@{*/
#define CSR_ETRIGGER_ACTION_OFFSET          (0)
#define CSR_ETRIGGER_ACTION_LENGTH          (6)
#define CSR_ETRIGGER_ACTION                 (UINT64_C(0x3F) << CSR_ETRIGGER_ACTION_OFFSET)
/**@}*/
#endif

#define DMI_DMSTATUS                        (0x11)

/** @name DMI_DMSTATUS_IMPEBREAK

If 1, then there is an implicit @c ebreak instruction at the
non-existent word immediately after the Program Buffer. This saves
the debugger from having to write the @c ebreak itself, and
allows the Program Buffer to be one word smaller.

This must be 1 when @c progbufsize is 1.
*/
/**@{*/
#define DMI_DMSTATUS_IMPEBREAK_OFFSET       (22)
#if 0
#define DMI_DMSTATUS_IMPEBREAK_LENGTH       (1)
#endif
#define DMI_DMSTATUS_IMPEBREAK              (UINT32_C(0x1) << DMI_DMSTATUS_IMPEBREAK_OFFSET)
/**@}*/

/*
* This field is 1 when all currently selected harts have been reset but the reset has not been acknowledged.
 */
#define DMI_DMSTATUS_ALLHAVERESET_OFFSET    (19)
#if 0
#define DMI_DMSTATUS_ALLHAVERESET_LENGTH    (1)
#endif
#define DMI_DMSTATUS_ALLHAVERESET           (UINT32_C(0x1) << DMI_DMSTATUS_ALLHAVERESET_OFFSET)
/*
* This field is 1 when any currently selected hart has been reset but the reset has not been acknowledged.
 */
#define DMI_DMSTATUS_ANYHAVERESET_OFFSET    (18)
#define DMI_DMSTATUS_ANYHAVERESET_LENGTH    (1)
#define DMI_DMSTATUS_ANYHAVERESET           (UINT32_C(0x1) << DMI_DMSTATUS_ANYHAVERESET_OFFSET)
/*
* This field is 1 when all currently selected harts have acknowledged
* the previous resume request.
 */
#define DMI_DMSTATUS_ALLRESUMEACK_OFFSET    (17)
#if 0
#define DMI_DMSTATUS_ALLRESUMEACK_LENGTH    (1)
#endif
#define DMI_DMSTATUS_ALLRESUMEACK           (UINT32_C(0x1) << DMI_DMSTATUS_ALLRESUMEACK_OFFSET)

/**	This field is 1 when any currently selected hart has acknowledged
	the previous resume request.
 */
/**@{*/
#define DMI_DMSTATUS_ANYRESUMEACK_OFFSET    (16)
#if 0
#define DMI_DMSTATUS_ANYRESUMEACK_LENGTH    (1)
#endif
#define DMI_DMSTATUS_ANYRESUMEACK           (UINT32_C(0x1) << DMI_DMSTATUS_ANYRESUMEACK_OFFSET)
/**@}*/

/** This field is 1 when all currently selected harts do not exist in this system. */
/**@{*/
#define DMI_DMSTATUS_ALLNONEXISTENT_OFFSET  (15)
#if 0
#define DMI_DMSTATUS_ALLNONEXISTENT_LENGTH  (1)
#endif
#define DMI_DMSTATUS_ALLNONEXISTENT         (UINT32_C(0x1) << DMI_DMSTATUS_ALLNONEXISTENT_OFFSET)
/**@}*/

/** This field is 1 when any currently selected hart does not exist in this system. */
/**@{*/
#define DMI_DMSTATUS_ANYNONEXISTENT_OFFSET  (14)
#if 0
#define DMI_DMSTATUS_ANYNONEXISTENT_LENGTH  (1)
#endif
#define DMI_DMSTATUS_ANYNONEXISTENT         (UINT32_C(0x1) << DMI_DMSTATUS_ANYNONEXISTENT_OFFSET)
/**@}*/

/** This field is 1 when all currently selected harts are unavailable. */
/**@{*/
#define DMI_DMSTATUS_ALLUNAVAIL_OFFSET      (13)
#if 0
#define DMI_DMSTATUS_ALLUNAVAIL_LENGTH      (1)
#endif
#define DMI_DMSTATUS_ALLUNAVAIL             (UINT32_C(0x1) << DMI_DMSTATUS_ALLUNAVAIL_OFFSET)
/**@}*/

/** This field is 1 when any currently selected hart is unavailable. */
/**@{*/
#define DMI_DMSTATUS_ANYUNAVAIL_OFFSET      (12)
#if 0
#define DMI_DMSTATUS_ANYUNAVAIL_LENGTH      (1)
#endif
#define DMI_DMSTATUS_ANYUNAVAIL             (UINT32_C(0x1) << DMI_DMSTATUS_ANYUNAVAIL_OFFSET)
/**@}*/

/** This field is 1 when all currently selected harts are running. */
/**@{*/
#define DMI_DMSTATUS_ALLRUNNING_OFFSET      (11)
#if 0
#define DMI_DMSTATUS_ALLRUNNING_LENGTH      (1)
#endif
#define DMI_DMSTATUS_ALLRUNNING             (UINT32_C(0x1) << DMI_DMSTATUS_ALLRUNNING_OFFSET)
/**@}*/

/** This field is 1 when any currently selected hart is running. */
/**@{*/
#define DMI_DMSTATUS_ANYRUNNING_OFFSET      (10)
#if 0
#define DMI_DMSTATUS_ANYRUNNING_LENGTH      (1)
#endif
#define DMI_DMSTATUS_ANYRUNNING             (UINT32_C(0x1) << DMI_DMSTATUS_ANYRUNNING_OFFSET)
/**@}*/

/** This field is 1 when all currently selected harts are halted. */
/**@{*/
#define DMI_DMSTATUS_ALLHALTED_OFFSET       (9)
#if 0
#define DMI_DMSTATUS_ALLHALTED_LENGTH       (1)
#endif
#define DMI_DMSTATUS_ALLHALTED              (UINT32_C(0x1) << DMI_DMSTATUS_ALLHALTED_OFFSET)
/**@}*/

/** This field is 1 when any currently selected hart is halted. */
/**@{*/
#define DMI_DMSTATUS_ANYHALTED_OFFSET       (8)
#if 0
#define DMI_DMSTATUS_ANYHALTED_LENGTH       (1)
#endif
#define DMI_DMSTATUS_ANYHALTED              (UINT32_C(0x1) << DMI_DMSTATUS_ANYHALTED_OFFSET)
/**@}*/

/**
	0 when authentication is required before using the DM.  1 when the
	authentication check has passed. On components that don't implement
	authentication, this bit must be preset as 1.
*/
/**@{*/
#define DMI_DMSTATUS_AUTHENTICATED_OFFSET   (7)
#if 0
#define DMI_DMSTATUS_AUTHENTICATED_LENGTH   (1)
#endif
#define DMI_DMSTATUS_AUTHENTICATED          (UINT32_C(0x1) << DMI_DMSTATUS_AUTHENTICATED_OFFSET)
/**@}*/

/**

	0: The authentication module is ready to process the next
	read/write to @c authdata.

	1: The authentication module is busy. Accessing @c authdata results
	in unspecified behavior.
	
	@c authbusy only becomes set in immediate response to an access to
	@c authdata.
*/
/**@{*/
#define DMI_DMSTATUS_AUTHBUSY_OFFSET        (6)
#if 0
#define DMI_DMSTATUS_AUTHBUSY_LENGTH        (1)
#endif
#define DMI_DMSTATUS_AUTHBUSY               (UINT32_C(0x1) << DMI_DMSTATUS_AUTHBUSY_OFFSET)
/**@}*/

#if 0
/**
	1 if this Debug Module supports halt-on-reset functionality
	controllable by the @c setresethaltreq and @c clrresethaltreq bits.
	0 otherwise.
*/
/**@{*/
#define DMI_DMSTATUS_HASRESETHALTREQ_OFFSET (5)
#define DMI_DMSTATUS_HASRESETHALTREQ_LENGTH (1)
#define DMI_DMSTATUS_HASRESETHALTREQ        (0x1U << DMI_DMSTATUS_HASRESETHALTREQ_OFFSET)
/**@}*/
#endif

/** @name DMI_DMSTATUS_DEVTREEVALID
	0: @c devtreeaddrzero--@c devtreeaddrthree hold information which
	is not relevant to the Device Tree.
	
	1: @c devtreeaddrzero--@c devtreeaddrthree registers hold the address of the
	Device Tree.
*/
/**@{*/
#define DMI_DMSTATUS_DEVTREEVALID_OFFSET    (4)
#if 0
#define DMI_DMSTATUS_DEVTREEVALID_LENGTH    (1)
#endif
#define DMI_DMSTATUS_DEVTREEVALID           (UINT32_C(0x1) << DMI_DMSTATUS_DEVTREEVALID_OFFSET)
/**@}*/


/** @name DMI_DMSTATUS_VERSION
	0: There is no Debug Module present.
	
	1: There is a Debug Module and it conforms to version 0.11 of this
	specification.
	
	2: There is a Debug Module and it conforms to version 0.13 of this
	specification.
	
	15: There is a Debug Module but it does not conform to any
	available version of this spec.
*/
/**@{*/
#define DMI_DMSTATUS_VERSION_OFFSET         (0)
#if 0
#define DMI_DMSTATUS_VERSION_LENGTH         (4)
#endif
#define DMI_DMSTATUS_VERSION                (UINT32_C(0xF) << DMI_DMSTATUS_VERSION_OFFSET)
/**@}*/
#define DMI_DMCONTROL                       (0x10)

/**	Writes the halt request bit for all currently selected harts.

	When set to 1, each selected hart will halt if it is not currently
	halted.
	
	Writing 1 or 0 has no effect on a hart which is already halted, but
	the bit must be cleared to 0 before the hart is resumed.
	
	Writes apply to the new value of @c hartsel and @c hasel.
*/
/**@{*/
#define DMI_DMCONTROL_HALTREQ_OFFSET        (31)
#if 0
#define DMI_DMCONTROL_HALTREQ_LENGTH        (1)
#endif
#define DMI_DMCONTROL_HALTREQ               (UINT32_C(0x1) << DMI_DMCONTROL_HALTREQ_OFFSET)
/**@}*/

/**	Writes the resume request bit for all currently selected harts.

	When set to 1, each selected hart will resume if it is currently
	halted.
	
	The resume request bit is ignored while the halt request bit is set.
	
	Writes apply to the new value of @c hartsel and @c hasel.
*/
/**@{*/
#define DMI_DMCONTROL_RESUMEREQ_OFFSET      (30)
#if 0
#define DMI_DMCONTROL_RESUMEREQ_LENGTH      (1)
#endif
#define DMI_DMCONTROL_RESUMEREQ             (0x1U << DMI_DMCONTROL_RESUMEREQ_OFFSET)
/**@}*/

/**	This optional field writes the reset bit for all the currently
	selected harts.
	To perform a reset the debugger writes 1, and then writes 0 to deassert the reset signal.
	
	If this feature is not implemented, the bit always stays 0, so
	after writing 1 the debugger can read the register back to see if
	the feature is supported.
	
	Writes apply to the new value of @c hartsel and @c hasel.
*/
/**@{*/
#define DMI_DMCONTROL_HARTRESET_OFFSET      (29)
#if 0
#define DMI_DMCONTROL_HARTRESET_LENGTH      (1)
#endif
#define DMI_DMCONTROL_HARTRESET             (UINT32_C(0x1) << DMI_DMCONTROL_HARTRESET_OFFSET)
/**@}*/

/**	Writing 1 to this bit clears the @c havereset bits for any selected harts.

	Writes apply to the new value of @c hartsel and @c hasel.
*/
/**@{*/
#define DMI_DMCONTROL_ACKHAVERESET_OFFSET   (28)
#if 0
#define DMI_DMCONTROL_ACKHAVERESET_LENGTH   (1)
#endif
#define DMI_DMCONTROL_ACKHAVERESET          (UINT32_C(0x1) << DMI_DMCONTROL_ACKHAVERESET_OFFSET)
/**@}*/

/**	Selects the  definition of currently selected harts.

	0: There is a single currently selected hart, that selected by @c hartsel.

	1: There may be multiple currently selected harts -- that selected by @c hartsel,
	plus those selected by the hart array mask register.
	
	An implementation which does not implement the hart array mask register
	must tie this field to 0. A debugger which wishes to use the hart array
	mask register feature should set this bit and read back to see if the functionality
	is supported.
*/
/**@{*/
#define DMI_DMCONTROL_HASEL_OFFSET          (26)
#if 0
#define DMI_DMCONTROL_HASEL_LENGTH          (1)
#endif
#define DMI_DMCONTROL_HASEL                 (UINT32_C(0x1) << DMI_DMCONTROL_HASEL_OFFSET)
/**@}*/

/**	The low 10 bits of @c hartsel: the DM-specific index of the hart to
	select. This hart is always part of the currently selected harts.
*/
/**@{*/
#define DMI_DMCONTROL_HARTSELLO_OFFSET      (16)
#define DMI_DMCONTROL_HARTSELLO_LENGTH      (10)
#define DMI_DMCONTROL_HARTSELLO             (UINT32_C(0x3FF) << DMI_DMCONTROL_HARTSELLO_OFFSET)
/**@}*/

/**	The high 10 bits of @c hartsel: the DM-specific index of the hart to
	select. This hart is always part of the currently selected harts.
*/
/**@{*/
#define DMI_DMCONTROL_HARTSELHI_OFFSET      (6)
#define DMI_DMCONTROL_HARTSELHI_LENGTH      (10)
#define DMI_DMCONTROL_HARTSELHI             (UINT32_C(0x3FF) << DMI_DMCONTROL_HARTSELHI_OFFSET)
/**@}*/

#if 0
/**	This optional field writes the halt-on-reset request bit for all
	currently selected harts.
	When set to 1, each selected hart will halt upon the next deassertion
	of its reset. The halt-on-reset request bit is not automatically
	cleared. The debugger must write to @c clrresethaltreq to clear it.
	
	Writes apply to the new value of @c hartsel and @c hasel.

	If @c hasresethaltreq is 0, this field is not implemented.
*/
/**@{*/
#define DMI_DMCONTROL_SETRESETHALTREQ_OFFSET (3)
#if 0
#define DMI_DMCONTROL_SETRESETHALTREQ_LENGTH (1)
#endif
#define DMI_DMCONTROL_SETRESETHALTREQ       (UINT32_C(0x1) << DMI_DMCONTROL_SETRESETHALTREQ_OFFSET)
/**@}*/

/**	This optional field clears the halt-on-reset request bit for all currently selected harts.

	Writes apply to the new value of @c hartsel and @c hasel.
*/
/**@{*/
#define DMI_DMCONTROL_CLRRESETHALTREQ_OFFSET (2)
#if 0
#define DMI_DMCONTROL_CLRRESETHALTREQ_LENGTH (1)
#endif
#define DMI_DMCONTROL_CLRRESETHALTREQ       (UINT32_C(0x1) << DMI_DMCONTROL_CLRRESETHALTREQ_OFFSET)
/**@}*/
#endif

/**	This bit controls the reset signal from the DM to the rest of the
	system.
	
	The signal should reset every part of the system, including
	every hart, except for the DM and any logic required to access the DM.
	To perform a system reset the debugger writes 1,
	and then writes 0 to deassert the reset.
*/
/**@{*/
#define DMI_DMCONTROL_NDMRESET_OFFSET       (1)
#if 0
#define DMI_DMCONTROL_NDMRESET_LENGTH       (1)
#endif
#define DMI_DMCONTROL_NDMRESET              (UINT32_C(0x1) << DMI_DMCONTROL_NDMRESET_OFFSET)
/**@}*/

/**	This bit serves as a reset signal for the Debug Module itself.

	0: The module's state, including authentication mechanism,
	takes its reset values (the @c dmactive bit is the only bit which can
	be written to something other than its reset value).
	
	1: The module functions normally.
	
	No other mechanism should exist that may result in resetting the
	Debug Module after power up, including the platform's system reset
	or Debug Transport reset signals.
	
	A debugger may pulse this bit low to get the Debug Module into a
	known state.
	
	Implementations may use this bit to aid debugging, for example by
	preventing the Debug Module from being power gated while debugging
	is active.
*/
/**@{*/
#define DMI_DMCONTROL_DMACTIVE_OFFSET       (0)
#if 0
#define DMI_DMCONTROL_DMACTIVE_LENGTH       (1)
#endif
#define DMI_DMCONTROL_DMACTIVE              (UINT32_C(0x1) << DMI_DMCONTROL_DMACTIVE_OFFSET)
#define DMI_HARTINFO                        (0x12)
/**@}*/

/**	Number of @c dscratch registers available for the debugger
	to use during program buffer execution, starting from @c dscratchzero.
	The debugger can make no assumptions about the contents of these
	registers between commands.
*/
/**@{*/
#define DMI_HARTINFO_NSCRATCH_OFFSET        (20)
#if 0
#define DMI_HARTINFO_NSCRATCH_LENGTH        (4)
#endif
#define DMI_HARTINFO_NSCRATCH               (UINT32_C(0xF) << DMI_HARTINFO_NSCRATCH_OFFSET)
/**@}*/

/**
	0: The @c data registers are shadowed in the hart by CSR
	registers. Each CSR register is MXLEN bits in size, and corresponds
	to a single argument, per Table~\ref{tab:datareg}.
	
	1: The @c data registers are shadowed in the hart's memory map.
	Each register takes up 4 bytes in the memory map.
*/
/**@{*/
#define DMI_HARTINFO_DATAACCESS_OFFSET      (16)
#if 0
#define DMI_HARTINFO_DATAACCESS_LENGTH      (1)
#endif
#define DMI_HARTINFO_DATAACCESS             (UINT32_C(0x1) << DMI_HARTINFO_DATAACCESS_OFFSET)
/**@}*/

/**
	If @c dataaccess is 0: Number of CSR registers dedicated to
	shadowing the @c data registers.
	
	If @c dataaccess is 1: Number of 32-bit words in the memory map
	dedicated to shadowing the @c data registers.
	
	Since there are at most 12 @c data registers, the value in this
	register must be 12 or smaller.
*/
/**@{*/
#define DMI_HARTINFO_DATASIZE_OFFSET        (12)
#if 0
#define DMI_HARTINFO_DATASIZE_LENGTH        (4)
#endif
#define DMI_HARTINFO_DATASIZE               (UINT32_C(0xF) << DMI_HARTINFO_DATASIZE_OFFSET)
/**@}*/

/**
	If @c dataaccess is 0: The number of the first CSR dedicated to
	shadowing the @c data registers.
	
	If @c dataaccess is 1: Signed address of RAM where the @c data
	registers are shadowed, to be used to access relative to @c zero.
*/
/**@{*/
#define DMI_HARTINFO_DATAADDR_OFFSET        (0)
#if 0
#define DMI_HARTINFO_DATAADDR_LENGTH        (12)
#endif
#define DMI_HARTINFO_DATAADDR               (UINT32_C(0xFFF) << DMI_HARTINFO_DATAADDR_OFFSET)
#if 0
#define DMI_HAWINDOWSEL                     (0x14)
#endif
/**@}*/

/**	The high bits of this field may be tied to 0, depending on how large
	the array mask register is.
	Eg. on a system with 48 harts only bit 0 of this field may actually be writable.
*/
/**@{*/
#if 0
#define DMI_HAWINDOWSEL_HAWINDOWSEL_OFFSET  (0)
#define DMI_HAWINDOWSEL_HAWINDOWSEL_LENGTH  (15)
#define DMI_HAWINDOWSEL_HAWINDOWSEL         (UINT32_C(0x7FFF) << DMI_HAWINDOWSEL_HAWINDOWSEL_OFFSET)
#define DMI_HAWINDOW                        (0x15)
#define DMI_HAWINDOW_MASKDATA_OFFSET        (0)
#define DMI_HAWINDOW_MASKDATA_LENGTH        (32)
#define DMI_HAWINDOW_MASKDATA               (UINT32_C(0xFFFFFFFF) << DMI_HAWINDOW_MASKDATA_OFFSET)
#endif
#define DMI_ABSTRACTCS                      (0x16)
/**@}*/

/**	Size of the Program Buffer, in 32-bit words. Valid sizes are 0 - 16.*/
/**@{*/
#define DMI_ABSTRACTCS_PROGBUFSIZE_OFFSET   (24)
#if 0
#define DMI_ABSTRACTCS_PROGBUFSIZE_LENGTH   (5)
#endif
#define DMI_ABSTRACTCS_PROGBUFSIZE          (UINT32_C(0x1F) << DMI_ABSTRACTCS_PROGBUFSIZE_OFFSET)
/**@}*/

/**
	1: An abstract command is currently being executed.
	
	This bit is set as soon as @c command is written, and is
	not cleared until that command has completed.
*/
/**@{*/
#define DMI_ABSTRACTCS_BUSY_OFFSET          (12)
#if 0
#define DMI_ABSTRACTCS_BUSY_LENGTH          (1)
#endif
#define DMI_ABSTRACTCS_BUSY                 (UINT32_C(0x1) << DMI_ABSTRACTCS_BUSY_OFFSET)
/**@}*/

/**	Gets set if an abstract command fails.

	The bits in this field remain set until
	they are cleared by writing 1 to them. No abstract command is
	started until the value is reset to 0.
	
	0 (none): No error.
	
	1 (busy): An abstract command was executing while @c command,
	@c abstractcs, @c abstractauto was written, or when one
	of the @c data or @c progbuf registers was read or written.
	
	2 (not supported): The requested command is not supported,
	regardless of whether the hart is running or not.
	
	3 (exception): An exception occurred while executing the command
	(eg. while executing the Program Buffer).
	
	4 (halt/resume): The abstract command couldn't execute because the
	hart wasn't in the required state (running/halted).
	
	7 (other): The command failed for another reason.
*/
/**@{*/
#define DMI_ABSTRACTCS_CMDERR_OFFSET        (8)
#if 0
#define DMI_ABSTRACTCS_CMDERR_LENGTH        (3)
#endif
#define DMI_ABSTRACTCS_CMDERR               (UINT32_C(0x7) << DMI_ABSTRACTCS_CMDERR_OFFSET)
/**@}*/

/**	Number of @c data registers that are implemented as part of the
	abstract command interface.
	
	Valid sizes are 0 - 12.
*/
/**@{*/
#define DMI_ABSTRACTCS_DATACOUNT_OFFSET     (0)
#if 0
#define DMI_ABSTRACTCS_DATACOUNT_LENGTH     (4)
#endif
#define DMI_ABSTRACTCS_DATACOUNT            (UINT32_C(0xF) << DMI_ABSTRACTCS_DATACOUNT_OFFSET)
#define DMI_COMMAND                         (0x17)
/**@}*/

/**	The type determines the overall functionality of this abstract command. */
/**@{*/
#define DMI_COMMAND_CMDTYPE_OFFSET          (24)
#if 0
#define DMI_COMMAND_CMDTYPE_LENGTH          (8)
#endif
#define DMI_COMMAND_CMDTYPE                 (UINT32_C(0xFF) << DMI_COMMAND_CMDTYPE_OFFSET)
/**@}*/

/**	This field is interpreted in a command-specific manner, described for each abstract command. */
/**@{*/
#if 0
#define DMI_COMMAND_CONTROL_OFFSET          (0)
#define DMI_COMMAND_CONTROL_LENGTH          (24)
#define DMI_COMMAND_CONTROL                 (UINT32_C(0xFFFFFF) << DMI_COMMAND_CONTROL_OFFSET)
#endif
/**@}*/

#define DMI_ABSTRACTAUTO                    (0x18)

/** @name DMI_ABSTRACTAUTO_AUTOEXECPROGBUF
	When a bit in this field is 1, read or write accesses to the corresponding @c progbuf word
	cause the command in @c command to be executed again.
*/
/**@{*/
#define DMI_ABSTRACTAUTO_AUTOEXECPROGBUF_OFFSET (16)
#if 0
#define DMI_ABSTRACTAUTO_AUTOEXECPROGBUF_LENGTH (16)
#endif
#define DMI_ABSTRACTAUTO_AUTOEXECPROGBUF    (UINT32_C(0xFFFF) << DMI_ABSTRACTAUTO_AUTOEXECPROGBUF_OFFSET)
/**@}*/

/**	When a bit in this field is 1, read or write accesses to the corresponding @c data word
	cause the command in @c command to be executed again.
*/
/**@{*/
#define DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET (0)
#if 0
#define DMI_ABSTRACTAUTO_AUTOEXECDATA_LENGTH (12)
#endif
#define DMI_ABSTRACTAUTO_AUTOEXECDATA       (UINT32_C(0xFFF) << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET)
#if 0
#define DMI_DEVTREEADDR0                    (0x19)
#define DMI_DEVTREEADDR0_ADDR_OFFSET        (0)
#define DMI_DEVTREEADDR0_ADDR_LENGTH        (32)
#define DMI_DEVTREEADDR0_ADDR               (UINT32_C(0xFFFFFFFF) << DMI_DEVTREEADDR0_ADDR_OFFSET)
#define DMI_DEVTREEADDR1                    (0x1A)
#define DMI_DEVTREEADDR2                    (0x1B)
#define DMI_DEVTREEADDR3                    (0x1C)
#define DMI_NEXTDM                          (0x1D)
#define DMI_NEXTDM_ADDR_OFFSET              (0)
#define DMI_NEXTDM_ADDR_LENGTH              (32)
#define DMI_NEXTDM_ADDR                     (UINT32_C(0xFFFFFFFF) << DMI_NEXTDM_ADDR_OFFSET)
#endif
#define DMI_DATA0                           (0x04)
#if 0
#define DMI_DATA0_DATA_OFFSET               (0)
#define DMI_DATA0_DATA_LENGTH               (32)
#define DMI_DATA0_DATA                      (UINT32_C(0xFFFFFFFF) << DMI_DATA0_DATA_OFFSET)
#define DMI_DATA11                          (0x0F)
#endif

/**@{*/
#define DMI_PROGBUF0                        (0x20)
#if 0
#define DMI_PROGBUF0_DATA_OFFSET            (0)
#define DMI_PROGBUF0_DATA_LENGTH            (32)
#define DMI_PROGBUF0_DATA                   (UINT32_C(0xFFFFFFFF) << DMI_PROGBUF0_DATA_OFFSET)
#define DMI_PROGBUF15                       (0x2F)
#endif
/**@}*/

/**@{*/
#define DMI_AUTHDATA                        (0x30)
#if 0
#define DMI_AUTHDATA_DATA_OFFSET            (0)
#define DMI_AUTHDATA_DATA_LENGTH            (32)
#define DMI_AUTHDATA_DATA                   (UINT32_C(0xFFFFFFFF) << DMI_AUTHDATA_DATA_OFFSET)
#endif
/**@}*/

#define DMI_HALTSUM0                        (0x40)
#if 0
#define DMI_HALTSUM0_HALTSUM0_OFFSET        (0)
#define DMI_HALTSUM0_HALTSUM0_LENGTH        (32)
#define DMI_HALTSUM0_HALTSUM0               (UINT32_C(0xFFFFFFFF) << DMI_HALTSUM0_HALTSUM0_OFFSET)
#endif
#define DMI_HALTSUM1                        (0x13)
#if 0
#define DMI_HALTSUM1_HALTSUM1_OFFSET        (0)
#define DMI_HALTSUM1_HALTSUM1_LENGTH        (32)
#define DMI_HALTSUM1_HALTSUM1               (UINT32_C(0xFFFFFFFF) << DMI_HALTSUM1_HALTSUM1_OFFSET)
#define DMI_HALTSUM2                        (0x34)
#define DMI_HALTSUM2_HALTSUM2_OFFSET        (0)
#define DMI_HALTSUM2_HALTSUM2_LENGTH        (32)
#define DMI_HALTSUM2_HALTSUM2               (UINT32_C(0xFFFFFFFF) << DMI_HALTSUM2_HALTSUM2_OFFSET)
#define DMI_HALTSUM3                        (0x35)
#define DMI_HALTSUM3_HALTSUM3_OFFSET        (0)
#define DMI_HALTSUM3_HALTSUM3_LENGTH        (32)
#define DMI_HALTSUM3_HALTSUM3               (UINT32_C(0xFFFFFFFF) << DMI_HALTSUM3_HALTSUM3_OFFSET)
#endif
/**@}*/

/**	Accesses bits 127:96 of the physical address in @c sbaddress
	(if the system address bus is that wide).
*/
/**@{*/
#define DMI_SBADDRESS3                      (0x37)
#if 0
#define DMI_SBADDRESS3_ADDRESS_OFFSET       (0)
#define DMI_SBADDRESS3_ADDRESS_LENGTH       (32)
#define DMI_SBADDRESS3_ADDRESS              (UINT32_C(0xFFFFFFFF) << DMI_SBADDRESS3_ADDRESS_OFFSET)
#endif
/**@}*/

/** @name DMI_SBCS_SBVERSION
	0: The System Bus interface conforms to mainline drafts of this
	spec older than 1 January, 2018.
	
	1: The System Bus interface conforms to this version of the spec.
	
	Other values are reserved for future versions.
*/
/**@{*/
#define DMI_SBCS                            (0x38)
#define DMI_SBCS_SBVERSION_OFFSET           (29)
#if 0
#define DMI_SBCS_SBVERSION_LENGTH           (3)
#endif
#define DMI_SBCS_SBVERSION                  (UINT32_C(0x7) << DMI_SBCS_SBVERSION_OFFSET)
/**@}*/

/**	Set when the debugger attempts to read data while a read is in
	progress, or when the debugger initiates a new access while one is
	already in progress (while @c sbbusy is set).
	
	It remains set until it's explicitly cleared by the debugger.
	
	While this field is non-zero, no more system bus accesses can be
	initiated by the Debug Module.
*/
/**@{*/
#define DMI_SBCS_SBBUSYERROR_OFFSET         (22)
#if 0
#define DMI_SBCS_SBBUSYERROR_LENGTH         (1)
#endif
#define DMI_SBCS_SBBUSYERROR                (UINT32_C(0x1) << DMI_SBCS_SBBUSYERROR_OFFSET)
/**@}*/

/**	When 1, indicates the system bus master is busy.

	(Whether the system bus itself is busy is related, but not the same thing.)
	This bit goes high immediately when a read or write is requested for any
	reason, and does not go low until the access is fully completed.

	Writes to @c sbcs while @c sbbusy is high result in undefined
	behavior.  A debugger must not write to @c sbcs until it reads
	@c sbbusy as 0.
*/
/**@{*/
#define DMI_SBCS_SBBUSY_OFFSET              (21)
#if 0
#define DMI_SBCS_SBBUSY_LENGTH              (1)
#endif
#define DMI_SBCS_SBBUSY                     (UINT32_C(0x1) << DMI_SBCS_SBBUSY_OFFSET)
/**@}*/

/**
	When 1, every write to @c sbaddresszero automatically triggers a
	system bus read at the new address.
*/
/**@{*/
#define DMI_SBCS_SBREADONADDR_OFFSET        (20)
#if 0
#define DMI_SBCS_SBREADONADDR_LENGTH        (1)
#endif
#define DMI_SBCS_SBREADONADDR               (UINT32_C(0x1) << DMI_SBCS_SBREADONADDR_OFFSET)
/**@}*/

/**	Select the access size to use for system bus accesses.

	0: 8-bit
	
	1: 16-bit
	
	2: 32-bit
	
	3: 64-bit

	4: 128-bit
	
	If @c sbaccess has an unsupported value when the DM starts a bus
	access, the access is not performed and @c sberror is set to 3.
*/
/**@{*/
#define DMI_SBCS_SBACCESS_OFFSET            (17)
#if 0
#define DMI_SBCS_SBACCESS_LENGTH            (3)
#endif
#define DMI_SBCS_SBACCESS                   (UINT32_C(0x7) << DMI_SBCS_SBACCESS_OFFSET)
/**@}*/

/**
	When 1, @c sbaddress is incremented by the access size
	(in bytes) selected in @c sbaccess after every system bus access.
*/
/**@{*/
#define DMI_SBCS_SBAUTOINCREMENT_OFFSET     (16)
#if 0
#define DMI_SBCS_SBAUTOINCREMENT_LENGTH     (1)
#endif
#define DMI_SBCS_SBAUTOINCREMENT            (UINT32_C(0x1) << DMI_SBCS_SBAUTOINCREMENT_OFFSET)
/**@}*/

/**
	When 1, every read from @c sbdatazero automatically triggers a
	system bus read at the (possibly auto-incremented) address.
*/
/**@{*/
#define DMI_SBCS_SBREADONDATA_OFFSET        (15)
#if 0
#define DMI_SBCS_SBREADONDATA_LENGTH        (1)
#endif
#define DMI_SBCS_SBREADONDATA               (UINT32_C(0x1) << DMI_SBCS_SBREADONDATA_OFFSET)
/**@}*/

/**
	When the Debug Module's system bus
	master causes a bus error, this field gets set.
	The bits in this field remain set until they are cleared by writing 1 to them.
	While this field is non-zero, no more system bus accesses can be
	initiated by the Debug Module.
	
	An implementation may report "Other" (7) for any error condition.
	
	0: There was no bus error.
	
	1: There was a timeout.
	
	2: A bad address was accessed.
	
	3: There was an alignment error.
	
	4: An access of unsupported size was requested.
	
	7: Other.
*/
/**@{*/
#define DMI_SBCS_SBERROR_OFFSET             (12)
#if 0
#define DMI_SBCS_SBERROR_LENGTH             (3)
#endif
#define DMI_SBCS_SBERROR                    (UINT32_C(0x7) << DMI_SBCS_SBERROR_OFFSET)
/**@}*/

/**	Width of system bus addresses in bits.

	(0 indicates there is no bus access support.)
*/
/**@{*/
#define DMI_SBCS_SBASIZE_OFFSET             (5)
#if 0
#define DMI_SBCS_SBASIZE_LENGTH             (7)
#endif
#define DMI_SBCS_SBASIZE                    (UINT32_C(0x7F) << DMI_SBCS_SBASIZE_OFFSET)
/**@}*/

/** 1 when 128-bit system bus accesses are supported.*/
/**@{*/
#define DMI_SBCS_SBACCESS128_OFFSET         (4)
#if 0
#define DMI_SBCS_SBACCESS128_LENGTH         (1)
#endif
#define DMI_SBCS_SBACCESS128                (UINT32_C(0x1) << DMI_SBCS_SBACCESS128_OFFSET)
/**@}*/

/** 1 when 64-bit system bus accesses are supported.*/
/**@{*/
#define DMI_SBCS_SBACCESS64_OFFSET          (3)
#if 0
#define DMI_SBCS_SBACCESS64_LENGTH          (1)
#endif
#define DMI_SBCS_SBACCESS64                 (UINT32_C(0x1) << DMI_SBCS_SBACCESS64_OFFSET)
/**@}*/

/** 1 when 32-bit system bus accesses are supported. */
/**@{*/
#define DMI_SBCS_SBACCESS32_OFFSET          (2)
#if 0
#define DMI_SBCS_SBACCESS32_LENGTH          (1)
#endif
#define DMI_SBCS_SBACCESS32                 (UINT32_C(0x1) << DMI_SBCS_SBACCESS32_OFFSET)
/**@}*/

/** 1 when 16-bit system bus accesses are supported. */
/**@{*/
#define DMI_SBCS_SBACCESS16_OFFSET          (1)
#if 0
#define DMI_SBCS_SBACCESS16_LENGTH          (1)
#endif
#define DMI_SBCS_SBACCESS16                 (UINT32_C(0x1) << DMI_SBCS_SBACCESS16_OFFSET)
/**@}*/

/** 1 when 8-bit system bus accesses are supported. */
/**@{*/
#define DMI_SBCS_SBACCESS8_OFFSET           (0)
#if 0
#define DMI_SBCS_SBACCESS8_LENGTH           (1)
#endif
#define DMI_SBCS_SBACCESS8                  (UINT32_C(0x1) << DMI_SBCS_SBACCESS8_OFFSET)
/**@}*/

/** Accesses bits 31:0 of the physical address in @c sbaddress. */
/**@{*/
#define DMI_SBADDRESS0                      (0x39)
#if 0
#define DMI_SBADDRESS0_ADDRESS_OFFSET       (0)
#define DMI_SBADDRESS0_ADDRESS_LENGTH       (32)
#define DMI_SBADDRESS0_ADDRESS              (UINT32_C(0xFFFFFFFF) << DMI_SBADDRESS0_ADDRESS_OFFSET)
#endif
/**@}*/

/**	Accesses bits 63:32 of the physical address in @c sbaddress
	(if the system address bus is that wide).
*/
/**@{*/
#define DMI_SBADDRESS1                      (0x3A)
#if 0
#define DMI_SBADDRESS1_ADDRESS_OFFSET       (0)
#define DMI_SBADDRESS1_ADDRESS_LENGTH       (32)
#define DMI_SBADDRESS1_ADDRESS              (UINT32_C(0xFFFFFFFF) << DMI_SBADDRESS1_ADDRESS_OFFSET)
#endif
/**@}*/

/**	Accesses bits 95:64 of the physical address in @c sbaddress
	(if the system address bus is that wide).
*/
/**@{*/
#define DMI_SBADDRESS2                      (0x3B)
#if 0
#define DMI_SBADDRESS2_ADDRESS_OFFSET       (0)
#define DMI_SBADDRESS2_ADDRESS_LENGTH       (32)
#define DMI_SBADDRESS2_ADDRESS              (UINT32_C(0xFFFFFFFF) << DMI_SBADDRESS2_ADDRESS_OFFSET)
#endif
/**@}*/

/** Accesses bits 31:0 of @c sbdata. */
/**@{*/
#define DMI_SBDATA0                         (0x3C)
#if 0
#define DMI_SBDATA0_DATA_OFFSET             (0)
#define DMI_SBDATA0_DATA_LENGTH             (32)
#define DMI_SBDATA0_DATA                    (UINT32_C(0xFFFFFFFF) << DMI_SBDATA0_DATA_OFFSET)
#endif
/**@}*/

/** Accesses bits 63:32 of @c sbdata (if the system bus is that wide). */
/**@{*/
#define DMI_SBDATA1                         (0x3D)
#if 0
#define DMI_SBDATA1_DATA_OFFSET             (0)
#define DMI_SBDATA1_DATA_LENGTH             (32)
#define DMI_SBDATA1_DATA                    (UINT32_C(0xFFFFFFFF) << DMI_SBDATA1_DATA_OFFSET)
#endif
/**@}*/

/**	Accesses bits 95:64 of @c sbdata (if the system bus is that wide).*/
/**@{*/
#define DMI_SBDATA2                         (0x3E)
#if 0
#define DMI_SBDATA2_DATA_OFFSET             (0)
#define DMI_SBDATA2_DATA_LENGTH             (32)
#define DMI_SBDATA2_DATA                    (UINT32_C(0xFFFFFFFF) << DMI_SBDATA2_DATA_OFFSET)
#endif
/**@}*/

/**	Accesses bits 127:96 of @c sbdata (if the system bus is that wide).*/
/**@{*/
#define DMI_SBDATA3                         (0x3F)
#if 0
#define DMI_SBDATA3_DATA_OFFSET             (0)
#define DMI_SBDATA3_DATA_LENGTH             (32)
#define DMI_SBDATA3_DATA                    (UINT32_C(0xFFFFFFFF) << DMI_SBDATA3_DATA_OFFSET)
#define SHORTNAME                           (0x123)
#endif
/**@}*/

#if 0
/** Description of what this field is used for. */
/**@{*/
#define SHORTNAME_FIELD_OFFSET              (0)
#define SHORTNAME_FIELD_LENGTH              (8)
#define SHORTNAME_FIELD                     (UINT32_C(0xFF) << SHORTNAME_FIELD_OFFSET)
#define AC_ACCESS_REGISTER                  None
/**@}*/

/** This is 0 to indicate Access Register Command.*/
/**@{*/
#define AC_ACCESS_REGISTER_CMDTYPE_OFFSET   (24)
#define AC_ACCESS_REGISTER_CMDTYPE_LENGTH   (8)
#define AC_ACCESS_REGISTER_CMDTYPE          (UINT32_C(0xFF) << AC_ACCESS_REGISTER_CMDTYPE_OFFSET)
/**@}*/
#endif

/** @name AC_ACCESS_REGISTER_SIZE

2: Access the lowest 32 bits of the register.

3: Access the lowest 64 bits of the register.

4: Access the lowest 128 bits of the register.

If @c size specifies a size larger than the register's actual size,
then the access must fail. If a register is accessible, then reads of @c size
less than or equal to the register's actual size must be supported.

This field controls the Argument Width as referenced in
Table tab:datareg.
*/
/**@{*/
#define AC_ACCESS_REGISTER_SIZE_OFFSET      (20)
#if 0
#define AC_ACCESS_REGISTER_SIZE_LENGTH      (3)
#endif
#define AC_ACCESS_REGISTER_SIZE             (UINT32_C(0x7) << AC_ACCESS_REGISTER_SIZE_OFFSET)
/**@}*/

/** @name AC_ACCESS_REGISTER_POSTEXEC

	When 1, execute the program in the Program Buffer exactly once
	after performing the transfer, if any.
*/
/**@{ */
#define AC_ACCESS_REGISTER_POSTEXEC_OFFSET  (18)
#if 0
#define AC_ACCESS_REGISTER_POSTEXEC_LENGTH  (1)
#endif
#define AC_ACCESS_REGISTER_POSTEXEC         (UINT32_C(0x1) << AC_ACCESS_REGISTER_POSTEXEC_OFFSET)
/**@}*/

/** @name AC_ACCESS_REGISTER_TRANSFER

0: Don't do the operation specified by @c write.

1: Do the operation specified by @c write.

This bit can be used to just execute the Program Buffer without
having to worry about placing valid values into @c size or @c regno.
*/
/**@{*/
#define AC_ACCESS_REGISTER_TRANSFER_OFFSET  (17)
#if 0
#define AC_ACCESS_REGISTER_TRANSFER_LENGTH  (1)
#endif
#define AC_ACCESS_REGISTER_TRANSFER         (UINT32_C(0x1) << AC_ACCESS_REGISTER_TRANSFER_OFFSET)
/**@}*/

/** @name AC_ACCESS_REGISTER_WRITE

When @c transfer is set:

0: Copy data from the specified register into @c arg0 portion
of @c data.

1: Copy data from @c arg0 portion of @c data into the
specified register.
*/
/**@{*/
#define AC_ACCESS_REGISTER_WRITE_OFFSET     (16)
#if 0
#define AC_ACCESS_REGISTER_WRITE_LENGTH     (1)
#endif
#define AC_ACCESS_REGISTER_WRITE            (UINT32_C(0x1) << AC_ACCESS_REGISTER_WRITE_OFFSET)
/**@}*/

/** @name AC_ACCESS_REGISTER_REGNO
	@brief Number of the register to access, as described in Table tab:regno.

	@c dpc may be used as an alias for PC if this command is
	supported on a non-halted hart.
*/
/**@{*/
#define AC_ACCESS_REGISTER_REGNO_OFFSET     (0)
#if 0
#define AC_ACCESS_REGISTER_REGNO_LENGTH     (16)
#endif
#define AC_ACCESS_REGISTER_REGNO            (UINT32_C(0xFFFF) << AC_ACCESS_REGISTER_REGNO_OFFSET)
/**@}*/

#if 0
#define AC_QUICK_ACCESS                     None
#endif

#if 0
/** This is 1 to indicate Quick Access command.*/
/**@{*/
#define AC_QUICK_ACCESS_CMDTYPE_OFFSET      (24)
#define AC_QUICK_ACCESS_CMDTYPE_LENGTH      (8)
#define AC_QUICK_ACCESS_CMDTYPE             (0xffU << AC_QUICK_ACCESS_CMDTYPE_OFFSET)
#define VIRT_PRIV                           virtual
/**@}*/

/*
* Contains the privilege level the hart was operating in when Debug
* Mode was entered. The encoding is described in Table
* \ref{tab:privlevel}, and matches the privilege level encoding from
* the RISC-V Privileged ISA Specification. A user can write this
* value to change the hart's privilege level when exiting Debug Mode.
 */
#define VIRT_PRIV_PRV_OFFSET                (0)
#define VIRT_PRIV_PRV_LENGTH                (2)
#define VIRT_PRIV_PRV                       (0x3U << VIRT_PRIV_PRV_OFFSET)
#endif

#endif  /* TARGET_RISCV_DEBUG_DEFINES_H_ */
