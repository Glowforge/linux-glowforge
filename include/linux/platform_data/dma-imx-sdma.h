/**
 * Modifications to expose functionality to modules are
 * copyright (C) 2015-2018 Glowforge, Inc. <opensource@glowforge.com>
 */

#ifndef __MACH_MXC_SDMA_H__
#define __MACH_MXC_SDMA_H__

#include <linux/dmaengine.h>

/**
 * struct sdma_script_start_addrs - SDMA script start pointers
 *
 * start addresses of the different functions in the physical
 * address space of the SDMA engine.
 */
struct sdma_script_start_addrs {
	s32 ap_2_ap_addr;
	s32 ap_2_bp_addr;
	s32 ap_2_ap_fixed_addr;
	s32 bp_2_ap_addr;
	s32 loopback_on_dsp_side_addr;
	s32 mcu_interrupt_only_addr;
	s32 firi_2_per_addr;
	s32 firi_2_mcu_addr;
	s32 per_2_firi_addr;
	s32 mcu_2_firi_addr;
	s32 uart_2_per_addr;
	s32 uart_2_mcu_addr;
	s32 per_2_app_addr;
	s32 mcu_2_app_addr;
	s32 per_2_per_addr;
	s32 uartsh_2_per_addr;
	s32 uartsh_2_mcu_addr;
	s32 per_2_shp_addr;
	s32 mcu_2_shp_addr;
	s32 ata_2_mcu_addr;
	s32 mcu_2_ata_addr;
	s32 app_2_per_addr;
	s32 app_2_mcu_addr;
	s32 shp_2_per_addr;
	s32 shp_2_mcu_addr;
	s32 mshc_2_mcu_addr;
	s32 mcu_2_mshc_addr;
	s32 spdif_2_mcu_addr;
	s32 mcu_2_spdif_addr;
	s32 asrc_2_mcu_addr;
	s32 ext_mem_2_ipu_addr;
	s32 descrambler_addr;
	s32 dptc_dvfs_addr;
	s32 utra_addr;
	s32 ram_code_start_addr;
	/* End of v1 array */
	s32 mcu_2_ssish_addr;
	s32 ssish_2_mcu_addr;
	s32 hdmi_dma_addr;
	/* End of v2 array */
	s32 zcanfd_2_mcu_addr;
	s32 zqspi_2_mcu_addr;
	s32 mcu_2_ecspi_addr;
	/* End of v3 array */
};

/**
 * struct sdma_platform_data - platform specific data for SDMA engine
 *
 * @fw_name		The firmware name
 * @script_addrs	SDMA scripts addresses in SDMA ROM
 */
struct sdma_platform_data {
	char *fw_name;
	struct sdma_script_start_addrs *script_addrs;
};

/*
 * Mode/Count of data node descriptors - IPCv2
 */
struct sdma_mode_count {
	u32 count   : 16; /* size of the buffer pointed by this BD */
	u32 status  :  8; /* E,R,I,C,W,D status bits stored here */
	u32 command :  8; /* command mostlky used for channel 0 */
};

/*
 * Buffer descriptor
 */
struct sdma_buffer_descriptor {
	struct sdma_mode_count  mode;
	u32 buffer_addr;	/* address of the buffer described */
	u32 ext_buffer_addr;	/* extended buffer address */
} __attribute__ ((packed));

/**
 * struct sdma_channel_control - Channel control Block
 *
 * @current_bd_ptr	current buffer descriptor processed
 * @base_bd_ptr		first element of buffer descriptor array
 * @unused		padding. The SDMA engine expects an array of 128 byte
 *			control blocks
 */
struct sdma_channel_control {
	u32 current_bd_ptr;
	u32 base_bd_ptr;
	u32 unused[2];
} __attribute__ ((packed));

/**
 * struct sdma_state_registers - SDMA context for a channel
 *
 * @pc:		program counter
 * @t:		test bit: status of arithmetic & test instruction
 * @rpc:	return program counter
 * @sf:		source fault while loading data
 * @spc:	loop start program counter
 * @df:		destination fault while storing data
 * @epc:	loop end program counter
 * @lm:		loop mode
 */
struct sdma_state_registers {
	u32 pc     :14;
	u32 unused1: 1;
	u32 t      : 1;
	u32 rpc    :14;
	u32 unused0: 1;
	u32 sf     : 1;
	u32 spc    :14;
	u32 unused2: 1;
	u32 df     : 1;
	u32 epc    :14;
	u32 lm     : 2;
} __attribute__ ((packed));

/**
 * struct sdma_context_data - sdma context specific to a channel
 *
 * @channel_state:	channel state bits
 * @gReg:		general registers
 * @mda:		burst dma destination address register
 * @msa:		burst dma source address register
 * @ms:			burst dma status register
 * @md:			burst dma data register
 * @pda:		peripheral dma destination address register
 * @psa:		peripheral dma source address register
 * @ps:			peripheral dma status register
 * @pd:			peripheral dma data register
 * @ca:			CRC polynomial register
 * @cs:			CRC accumulator register
 * @dda:		dedicated core destination address register
 * @dsa:		dedicated core source address register
 * @ds:			dedicated core status register
 * @dd:			dedicated core data register
 */
struct sdma_context_data {
	struct sdma_state_registers  channel_state;
	u32  gReg[8];
	u32  mda;
	u32  msa;
	u32  ms;
	u32  md;
	u32  pda;
	u32  psa;
	u32  ps;
	u32  pd;
	u32  ca;
	u32  cs;
	u32  dda;
	u32  dsa;
	u32  ds;
	u32  dd;
	u32  scratch0;
	u32  scratch1;
	u32  scratch2;
	u32  scratch3;
	u32  scratch4;
	u32  scratch5;
	u32  scratch6;
	u32  scratch7;
} __attribute__ ((packed));

#define NUM_BD (int)(PAGE_SIZE / sizeof(struct sdma_buffer_descriptor))
#define SDMA_BD_MAX_CNT	0xfffc /* align with 4 bytes */

struct sdma_engine;

struct sdma_channel;

/**
 * sdma_engine_get() - returns a pointer to the global SDMA engine
 *
 * Return: pointer to the sdma_engine object
 */
struct sdma_engine *sdma_engine_get(void);

/**
 * sdma_get_channel() - returns a pointer to the numbered SDMA channel
 * @sdma:	pointer to the sdma_engine object
 * @channel:	channel number from 0-31
 *
 * Return: pointer to channel object, or NULL
 */
struct sdma_channel *sdma_get_channel(struct sdma_engine *sdma, int channel);

/**
 * sdma_request_channel() - enables use of a custom script on the given channel
 * @sdmac:	pointer to sdma_channel object
 * 
 * Return: 0 on success, nonzero otherwise
 */
int sdma_request_channel(struct sdma_channel *sdmac);

/**
 * sdma_set_channel_interrupt_callback() - sets a custom interrupt handler
 * @sdmac:	pointer to sdma_channel object
 * @int_cb:	callback function to register
 * @cb_param:	user-defined object passed when the callback is invoked
 *
 * Sets a function to be called when a custom SDMA script triggers an interrupt
 * (e.g. with a "done 3" instruction).
 * The function is executed in tasklet (atomic) context.
 */
void sdma_set_channel_interrupt_callback(struct sdma_channel *sdmac,
		dma_async_tx_callback int_cb,
		void *cb_param);

/**
 * sdma_set_channel_priority() - sets the channel's execution priority
 * @sdmac:	pointer to sdma_channel object
 * @priority:	priority, from 0 (disabled) to 7 (highest)
 *
 * Setting a nonzero priority may cause the channel's script to begin executing,
 * depending on how it is configured.
 * Priority 7 is used by channel 0 for loading scripts/context. Typically,
 * channel 0 should be the only channel with priority 7.
 *
 * Return: 0 on success, nonzero otherwise
 */
int sdma_set_channel_priority(struct sdma_channel *sdmac,
		unsigned int priority);

/**
 * sdma_config_ownership() - configures ownership of a channel
 * @sdmac:		pointer to sdma_channel object
 * @event_override:	if true, script can be triggered by an external event
 * @mcu_override:	if true, script can be triggered by the CPU
 * @dsp_override:	unused on i.MX6, always pass false
 *
 * Return: 0 on success, nonzero otherwise
 */
int sdma_config_ownership(struct sdma_channel *sdmac, bool event_override,
		bool mcu_override, bool dsp_override);

/**
 * sdma_setup_channel() - convenience function for setting channel ownership
 * @sdmac:	pointer to sdma_channel object
 * @external:	if true, script is triggered by an external event,
 *		if false, script is triggered by the CPU
 */
void sdma_setup_channel(struct sdma_channel *sdmac, bool external);

/**
 * sdma_enable_channel() - allows a channel to be run
 * @sdmac:	pointer to sdma_channel object
 */
void sdma_enable_channel(struct sdma_channel *sdmac);

/**
 * sdma_disable_channel() - prevents a channel from being run
 * @sdmac:	pointer to sdma_channel object
 */
void sdma_disable_channel(struct sdma_channel *sdmac);

/**
 * sdma_event_enable() - allows a channel to be triggered by the numbered event
 * @sdmac:	pointer to sdma_channel object
 * @event:	event number (see reference manual)
 */
void sdma_event_enable(struct sdma_channel *sdmac, unsigned int event);

/**
 * sdma_event_disable() - prevents a channel from being triggered by an event
 * @sdmac:	pointer to sdma_channel object
 * @event:	event number (see reference manual)
 */
void sdma_event_disable(struct sdma_channel *sdmac, unsigned int event);

/* address should be in program space (halfword addressing) */

/**
 * sdma_load_script() - copies script from ARM memory to SDMA memory
 * @sdma:	pointer to sdma_engine object
 * @buf:	start of script
 * @size:	size of script in bytes
 * @address:	destination address in SDMA program space
 *		(using halfword addressing)
 *
 * Return: 0 on success, nonzero on error
 */
int sdma_load_script(struct sdma_engine *sdma, void *buf, int size,
		u32 address);

/**
 * sdma_load_context_raw() - loads context for a channel running a custom script
 * @sdmac:	pointer to sdma_channel object
 * @context:	pointer to context data to load
 *
 * Loads arbitrary data into the channel's context RAM. Does not configure any
 * Linux DMA state.
 *
 * Return: 0 on success, nonzero on error
 */
int sdma_load_context_raw(struct sdma_channel *sdmac,
		struct sdma_context_data *context);

/**
 * sdma_load_partial_context() - writes a subset of a channel's context
 * @sdmac:		pointer to sdma_channel object
 * @context:		pointer to data to write
 * @byte_offset:	destination offset within the channel's context RAM
 *			(must be a multiple of 4 and less than 128)
 * @num_bytes:		number of bytes to copy into the channel's context RAM
 *			(must be > 0 and <= 128)
 *
 * Can be used to update a subset of a channel's registers while leaving others
 * undisturbed, e.g. to change a script's arguments while it is running without
 * overwriting internal state.
 * Since RAM loading is handled by channel 0, and channels cannot preempt each
 * other, the load operation is mutually exclusive with the channel's execution.
 * (i.e. a channel's registers will not change while its script is executing.)
 *
 * Example: to update a channel's entire context, use byte_offset=0 and
 * num_bytes=128.
 *
 * Return: 0 on success, nonzero on error
 */
int sdma_load_partial_context(struct sdma_channel *sdmac,
	struct sdma_context_data *context,
	u32 byte_offset,
	u32 num_bytes);

/* size should be a value in bytes */
/* address should be in data space (word addressing) */

/**
 * sdma_write_datamem() - writes data into the SDMA engine's address space
 * @sdma:	pointer to sdma_engine object
 * @buf:	data to write
 * @size:	number of bytes to write
 * @address:	destination offset, in 32-bit words, from the origin of SDMA
 *		address space
 *
 * Return: 0 on success, nonzero on error
 */
int sdma_write_datamem(struct sdma_engine *sdma, void *buf, int size,
	u32 address);

/**
 * sdma_fetch_datamem() - reads data from the SDMA engine's address space
 * @sdma:	pointer to sdma_engine object
 * @buf:	buffer to receive data
 * @size:	number of bytes to read
 * @address:	source offset, in 32-bit words, from the origin of SDMA address
 *		space
 *
 * Return: 0 on success, nonzero on error
 */
int sdma_fetch_datamem(struct sdma_engine *sdma, void *buf, int size,
	u32 address);

/**
 * sdma_fetch_partial_context() - reads a subset of a channel's context
 * @sdmac:		pointer to sdma_channel object
 * @buf:		buffer to receive data
 * @byte_offset:	source offset within the channel's context RAM
 *			(must be a multiple of 4 and less than 128)
 * @num_bytes:		number of bytes to read from the channel's context RAM
 *			(must be > 0 and <= 128)
 *
 * Since RAM loading is handled by channel 0, and channels cannot preempt each
 * other, the fetch operation is mutually exclusive with the channel's
 * execution. (i.e. the values will not be changing at the same time as they are
 * being read.)
 *
 * Example: to fetch a channel's entire context, use byte_offset=0 and
 * num_bytes=128.
 *
 * buf must be large enough to hold num_bytes of data.
 *
 * Return: 0 on success, nonzero on error
 */
int sdma_fetch_partial_context(struct sdma_channel *sdmac, void *buf,
    u32 byte_offset,
    u32 num_bytes);

/**
 * sdma_channel_context_base() - get the address of a channel's context RAM
 * @ch:		channel number, 0-31
 *
 * Return: offset from SDMA address space origin in 32-bit words
 */
u32 sdma_channel_context_base(int ch);

/**
 * sdma_print_context() - dump string representation of channel context values
 * @sdma:	pointer to sdma_engine object
 * @channel:	channel number, 0-31
 * @buf:	buffer to receive the string
 *
 * Prints a string representation of all channel registers and scratch memory
 * words. buf should be at least 512 bytes long. Useful for debugging.
 *
 * Return: result string length in bytes, or < 0 on error
 */
ssize_t sdma_print_context(struct sdma_engine *sdma, int channel, char *buf);

#endif /* __MACH_MXC_SDMA_H__ */
