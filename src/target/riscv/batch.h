#ifndef TARGET_RISCV_SCANS_H_
#define TARGET_RISCV_SCANS_H_

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

struct riscv_batch;
struct target;

/**	Allocates (or frees) a new scan set.

	"scans" is the maximum number of JTAG scans that can be issued to this object,
	and idle is the number of JTAG idle cycles between every real scan.
*/
struct riscv_batch *
	__attribute__((warn_unused_result))
	riscv_batch_alloc(struct target *target, size_t scans, size_t idle);

void
riscv_batch_free(struct riscv_batch *batch);

/** @brief Checks to see if this batch is full. */
bool
riscv_batch_full(struct riscv_batch const *const batch);

/** @brief Executes this scan batch. */
int
__attribute__((warn_unused_result))
riscv_batch_run(struct riscv_batch *batch);

/** @brief Adds a DMI write to this batch. */
void
riscv_batch_add_dmi_write(struct riscv_batch *batch,
	unsigned address,
	uint64_t data);

/**	DMI reads must be handled in two parts:
	the first one schedules a read and provides a key,
	the second one actually obtains the value of that read.
*/
/**@{*/
size_t
riscv_batch_add_dmi_read(struct riscv_batch *batch, unsigned address);
uint64_t
riscv_batch_get_dmi_read(struct riscv_batch const *batch, size_t key);
/**@}*/

#endif  /* TARGET_RISCV_SCANS_H_ */
