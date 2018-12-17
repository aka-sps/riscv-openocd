#include "batch.h"

#include "target/target.h"

#include "debug_defines.h"
#include "riscv.h"

#include "jtag/jtag.h"

enum riscv_scan_type_e {
	RISCV_SCAN_TYPE_INVALID,
	RISCV_SCAN_TYPE_NOP,
	RISCV_SCAN_TYPE_READ,
	RISCV_SCAN_TYPE_WRITE,
};

/**	A batch of multiple JTAG scans,
	which are grouped together to avoid the overhead of some JTAG adapters when sending single commands.
	This is designed to support block copies, as that's what we actually need to go fast.
*/
struct riscv_batch {
	struct target *target;

	size_t allocated_scans;
	size_t used_scans;

	size_t idle_count;

	uint8_t *data_out;
	uint8_t *data_in;
	struct scan_field *fields;

	/** In JTAG we scan out the previous value's output when performing a
	* scan.  This is a pain for users, so we just provide them the
	* illusion of not having to do this by eliding all but the last NOP.
	* */
	enum riscv_scan_type_e last_scan;

	/* The read keys. */
	size_t *read_keys;
	size_t read_keys_used;
};

bool
riscv_batch_full(struct riscv_batch const *const batch)
{
	return batch->allocated_scans < batch->used_scans + 4;
}

static void
riscv_batch_dump_field(struct scan_field const *const field)
{
	static const char *const op_string[] = {"-", "r", "w", "?"};
	static const char *const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	assert(field);
	assert(field->out_value != NULL);
	uint64_t const out_value = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned const out_op = get_field(out_value, DTM_DMI_OP);
	unsigned const out_data = get_field(out_value, DTM_DMI_DATA);
	unsigned const out_address = out_value >> DTM_DMI_ADDRESS_OFFSET;

	if (field->in_value) {
		uint64_t const in_value = buf_get_u64(field->in_value, 0, field->num_bits);
		unsigned const in_op = get_field(in_value, DTM_DMI_OP);
		unsigned const in_data = get_field(in_value, DTM_DMI_DATA);
		unsigned const in_address = in_value >> DTM_DMI_ADDRESS_OFFSET;

		LOG_DEBUG("%db %s %08x @%02x -> %s %08x @%02x",
			field->num_bits,
			op_string[out_op], out_data, out_address,
			status_string[in_op], in_data, in_address);
	} else {
		LOG_DEBUG("%db %s %08x @%02x -> ?",
			field->num_bits, op_string[out_op], out_data, out_address);
	}
}

struct riscv_batch *
	riscv_batch_alloc(struct target *const target,
		size_t const a_scans,
		size_t const idle)
{
	size_t const scans = a_scans + 4;
	struct riscv_batch const templ = {
		.target = target,
		.allocated_scans = scans,
		.used_scans = 0,
		.idle_count = idle,
		.last_scan = RISCV_SCAN_TYPE_INVALID,
		.read_keys_used = 0,
	};
	struct riscv_batch *p_out = malloc(sizeof *p_out);
	assert(p_out);
	*p_out = templ;

	p_out->data_out = malloc(scans * sizeof(uint64_t));
	assert(p_out->data_out);

	p_out->data_in  = malloc(scans * sizeof(uint64_t));
	assert(p_out->data_in);

	p_out->fields = malloc(scans * sizeof *p_out->fields);
	assert(p_out->fields);

	p_out->read_keys = malloc(scans * sizeof *p_out->read_keys);
	assert(p_out->read_keys);

	return p_out;
}

void
riscv_batch_free(struct riscv_batch *const batch)
{
	free(batch->fields);
	free(batch->data_in);
	free(batch->data_out);
	free(batch);
}

void
riscv_batch_add_dmi_write(struct riscv_batch *const batch,
	unsigned const address,
	uint64_t const data)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *const field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	uint8_t *const p_tmp = batch->data_out + batch->used_scans * sizeof(uint64_t);
	field->out_value = p_tmp;
	field->in_value  = batch->data_in  + batch->used_scans * sizeof(uint64_t);
	riscv_fill_dmi_write_u64(batch->target, p_tmp, address, data);
	riscv_fill_dmi_nop_u64(batch->target, field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_WRITE;
	++batch->used_scans;
}

uint64_t
riscv_batch_get_dmi_read(struct riscv_batch const *const batch,
	size_t const key)
{
	assert(key < batch->read_keys_used);
	size_t const index = batch->read_keys[key];
	assert(index <= batch->used_scans);
	uint8_t const *const base = batch->data_in + 8 * index;
	return
		(uint64_t)(base[0]) << 0 * CHAR_BIT |
		(uint64_t)(base[1]) << 1 * CHAR_BIT |
		(uint64_t)(base[2]) << 2 * CHAR_BIT |
		(uint64_t)(base[3]) << 3 * CHAR_BIT |
		(uint64_t)(base[4]) << 4 * CHAR_BIT |
		(uint64_t)(base[5]) << 5 * CHAR_BIT |
		(uint64_t)(base[6]) << 6 * CHAR_BIT |
		(uint64_t)(base[7]) << 7 * CHAR_BIT;
}

static void
riscv_batch_add_nop(struct riscv_batch *batch)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	uint8_t *p_tmp = batch->data_out + batch->used_scans * sizeof(uint64_t);
	field->in_value  = batch->data_in  + batch->used_scans * sizeof(uint64_t);
	field->out_value = p_tmp;
	riscv_fill_dmi_nop_u64(batch->target, p_tmp);
	riscv_fill_dmi_nop_u64(batch->target, field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_NOP;
	++batch->used_scans;
}

size_t
riscv_batch_add_dmi_read(struct riscv_batch *const batch,
	unsigned const address)
{
	assert(batch && batch->used_scans < batch->allocated_scans);
	struct scan_field *const field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	uint8_t *const p_tmp = batch->data_out + batch->used_scans * sizeof(uint64_t);
	field->out_value = p_tmp;
	field->in_value  = batch->data_in  + batch->used_scans * sizeof(uint64_t);
	riscv_fill_dmi_read_u64(batch->target, p_tmp, address);
	riscv_fill_dmi_nop_u64(batch->target, field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_READ;
	++batch->used_scans;

	/* FIXME We get the read response back on the next scan.  For now I'm
	 * just sticking a NOP in there, but this should be coalesced away. */
	riscv_batch_add_nop(batch);

	batch->read_keys[batch->read_keys_used] = batch->used_scans - 1;
	return ++batch->read_keys_used;
}

int
riscv_batch_run(struct riscv_batch *batch)
{
	assert(batch && batch->target);

	if (0 == batch->used_scans) {
		LOG_DEBUG("%s: Ignoring empty batch.",
			target_name(batch->target));
		return ERROR_OK;
	}

	keep_alive();

	riscv_batch_add_nop(batch);

	for (size_t i = 0; i < batch->used_scans; ++i) {
		jtag_add_dr_scan(batch->target->tap, 1, batch->fields + i, TAP_IDLE);
		if (0 < batch->idle_count)
			jtag_add_runtest(batch->idle_count, TAP_IDLE);
	}

	{
		int const error_code = jtag_execute_queue();

		if (ERROR_OK != error_code) {
			LOG_ERROR("%s: execute JTAG queue error: %d",
				jtag_tap_name(batch->target->tap), error_code);
			return error_code;
		}
	}

	for (size_t i = 0; i < batch->used_scans; ++i)
		riscv_batch_dump_field(batch->fields + i);

	return ERROR_OK;
}
