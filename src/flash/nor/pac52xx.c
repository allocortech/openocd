/***************************************************************************
 *   Copyright (C) 2019 by Steven Magee                                    *
 *   steven.magee@qorvo.com                                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "helper/binarybuffer.h"
#include <target/cortex_m.h>

// Base registers
#define PAC5XXX_FLASH_BASE                     		(0x00000000UL)

#define PAC5XXX_PERIPH_BASE 						(0x40000000UL)
#define PAC5XXX_MEMCTL_BASE 						(PAC5XXX_PERIPH_BASE + 0x20000)

// Memory Controller Register Addresses
#define PAC5XXX_MEMCTL_FLASHLOCK					(0x40020000UL)		// [31:0] - Must be written to correct key to allow write to page

#define PAC5XXX_MEMCTL_FLASHSTATUS					(0x40020004UL)
#define PAC5XXX_MEMCTL_FLASHSTATUS_PERASE			(1 << 1) 						// [1] - 1b: Erase in progress. 0b: erase finished or not in progress
#define PAC5XXX_MEMCTL_FLASHSTATUS_WRITE 			(1 << 0)						// [0] - 1b: Buffered write in progress. 0b: BW finished or not in progress

#define PAC5XXX_MEMCTL_FLASHPAGE					(0x40020008UL)		// [4:0] - Select page to write/erase

#define PAC5XXX_MEMCTL_FLASHPERASE					(0x40020014UL)	// [31:0] - Must be written to correct key to allow page erase

// Memory controller definitions
#define PAC5XXX_FLASH_LOCK_FLASHWRITE_KEY   		0xAAAAAAAA          			// Allow write to any FLASH pages not protected by RW bits
#define PAC5XXX_FLASH_LOCK_PERASE_KEY       		0xA5A55A5A          			// Allow write to FLASHPERASE register to erase FLASH pages

#define FLASH_ERASE_TIMEOUT 100
#define FLASH_WRITE_TIMEOUT 5

#define PAC5XXX_NUM_FLASH_PAGES 32


// Data structures
struct pac52xx_flash_bank 
{
	int probed;
};

// Methods

static inline int pac52xx_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
	return target_read_u32( target, PAC5XXX_MEMCTL_FLASHSTATUS, status );
}

static int pac52xx_wait_status_busy( struct flash_bank *bank, int timeout )
{
	uint32_t status;
	int retval = ERROR_OK;

	// Wait for the write or erase operation to complete or timeout
	for(;;) 
	{
		retval = pac52xx_get_flash_status( bank, &status );

		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("status: 0x%" PRIx32 "", status);

		// If PERASE and WRITE registers are both 0, operation is complete
		if( ( ( status & PAC5XXX_MEMCTL_FLASHSTATUS_PERASE ) && ( status & PAC5XXX_MEMCTL_FLASHSTATUS_WRITE ) ) == 0 )
			break;

		if (timeout-- <= 0) 
		{
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}

		alive_sleep(1);
	}

	return retval;
}

static int pac52xx_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int i;
	int retval = ERROR_OK;

	// Make sure target is halted
	if (bank->target->state != TARGET_HALTED) 
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	// Erase each page
	for( i = first; i <= last; ++i ) 
	{
		// Unlock flash write
		retval = target_write_u32( target, PAC5XXX_MEMCTL_FLASHLOCK, PAC5XXX_FLASH_LOCK_FLASHWRITE_KEY );
		if (retval != ERROR_OK)
			return retval;

		// Write the page to erase
		retval = target_write_u32( target, PAC5XXX_MEMCTL_FLASHPAGE, i );
		if (retval != ERROR_OK)
			return retval;

		// Unlock the erase page
		retval = target_write_u32( target, PAC5XXX_MEMCTL_FLASHPERASE, PAC5XXX_FLASH_LOCK_PERASE_KEY );
		if (retval != ERROR_OK)
			return retval;
		
		// Wait for erase to complete or time out
		retval = pac52xx_wait_status_busy( bank, FLASH_ERASE_TIMEOUT );
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}

static int pac52xx_protect(struct flash_bank *bank, int set, int first, int last)
{
	LOG_DEBUG("PROTECT NOT YET IMPLEMENTED");

	return ERROR_OK;
}

static int pac52xx_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint8_t *new_buffer = NULL;
    uint8_t additional_bytes;

	// Make sure target is halted
	if( bank->target->state != TARGET_HALTED ) 
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	// Check offset alignment - PAC52xx requires word alignment
	if( offset & 0x3 ) 
	{
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	// Check to see if bytecount breaks word alignment.
	// If so, create a copy of the buffer with additional padding bytes to satisfy alignment
	if (count & 0x3) 
	{
        additional_bytes = 4 - (count % 4);
		new_buffer = malloc(count + additional_bytes);

		if (new_buffer == NULL) 
		{
			LOG_ERROR("odd number of bytes to write and no memory for padding buffer");
			return ERROR_FAIL;
		}

		LOG_INFO("odd number of bytes to write, padding with 0xff");

		buffer = memcpy(new_buffer, buffer, count);
        for(uint8_t i = 0; i < additional_bytes; i++)
		{
            new_buffer[count++] = 0xff;
        }
	}

	uint32_t words_remaining = count / 4;

	int retval = ERROR_OK;

	// Write each 32-bit word
	while( words_remaining > 0 ) 
	{
		uint32_t value;

		// Fetch the current value to write
		memcpy( &value, buffer, sizeof(uint32_t) );

		// Write the flashlock key to allow writes
		retval = target_write_u32( target, PAC5XXX_MEMCTL_FLASHLOCK, PAC5XXX_FLASH_LOCK_FLASHWRITE_KEY );
		if (retval != ERROR_OK)
			break;

		// Write the 32-bit word to flash
		retval = target_write_u32(target, bank->base + offset, value);
		if (retval != ERROR_OK)
			break;

		// Wait for the write to complete
		retval = pac52xx_wait_status_busy( bank, FLASH_WRITE_TIMEOUT );
		if (retval != ERROR_OK)
			break;

		words_remaining--;
		buffer += 4;
		offset += 4;
	}

	if (new_buffer)
	{
		free(new_buffer);
	}

	return retval;
}

static int pac52xx_probe( struct flash_bank *bank )
{
	struct pac52xx_flash_bank *pac52xx_info = bank->driver_priv;

	int i;

	// Set base flash address
	uint32_t base_address 		= PAC5XXX_FLASH_BASE;

	// Set bank info
	pac52xx_info->probed 		= 0;

	// 32 1Kb pages
	int num_pages = 32;
	int page_size = 1024;

	// if sector structs exist, free them before proceeding
	if( bank->sectors ) 
	{
		free( bank->sectors );
		bank->sectors = NULL;
	}

	// Set bank info and allocate sector structures
	bank->base 			= base_address;
	bank->size 			= ( num_pages * page_size );
	bank->num_sectors 	= num_pages;
	bank->sectors 		= malloc( sizeof(struct flash_sector) * num_pages );

	// Set sector info
	for( i = 0; i < num_pages; i++ ) 
	{
		bank->sectors[i].offset 		= i * page_size;
		bank->sectors[i].size 			= page_size;
		bank->sectors[i].is_erased 		= -1;
		bank->sectors[i].is_protected 	= 1;
	}

	// Mark bank as probed
	pac52xx_info->probed = 1;

	return ERROR_OK;
}

static int pac52xx_auto_probe(struct flash_bank *bank)
{
	struct pac52xx_flash_bank *pac52xx_info = bank->driver_priv;

	if (pac52xx_info->probed)
	{
		// Already probed
		return ERROR_OK;
	}
	else
	{
		// Execute probe
		return pac52xx_probe( bank );
	}
}

static int pac52xx_protect_check(struct flash_bank *bank)
{
	// TODO: Read write protection status of all sectors
	LOG_DEBUG("PROTECT_CHECK NOT YET IMPLEMENTED");

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(pac52xx_flash_bank_command)
{
	struct pac52xx_flash_bank *pac52xx_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	pac52xx_info = malloc(sizeof(struct pac52xx_flash_bank));

	bank->driver_priv = pac52xx_info;
	pac52xx_info->probed = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(pac52xx_handle_info_command)
{
	LOG_DEBUG("INFO NOT YET IMPLEMENTED");
	return ERROR_OK;
}

COMMAND_HANDLER(pac525xx_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1) {
		command_print(CMD, "pac52xx mass_erase <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

    // Erase Bank: Pages 0 - Last; PAC52xx only has 1 bank=0
	retval = pac52xx_erase(bank, 0, PAC5XXX_NUM_FLASH_PAGES-1 );       // Args: bank, first_page, last_page
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "pac52xx mass erase complete");
	} else {
		command_print(CMD, "pac52xx mass erase failed");
	}

	return retval;
}

static const struct command_registration pac52xx_exec_command_handlers[] = 
{
	{
		.name = "info",
		.handler = pac52xx_handle_info_command,
		.mode = COMMAND_EXEC,
		.usage = "N/A",
		.help = "Get chip info"
	},
	{
		.name = "mass_erase",
		.handler = pac525xx_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire device flash.",
	},    
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration pac52xx_command_handlers[] = 
{
	{
		.name = "pac52xx",
		.mode = COMMAND_ANY,
		.help = "PAC52xx flash command group",
		.usage = "",
		.chain = pac52xx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver pac52xx_flash = 
{
	.name 				= "pac52xx",
	.commands 			= pac52xx_command_handlers,		//. Additional chip specific commands
	.flash_bank_command	= pac52xx_flash_bank_command,	//. Flash bank setup
	.erase 				= pac52xx_erase,				//. Erase sectors
	.protect 			= pac52xx_protect,				//. Enable or disable protection for sectors
	.write 				= pac52xx_write,				// Write X bytes to offset position
	.read 				= default_flash_read,			//. Read X bytes from offset position
	.probe 				= pac52xx_probe,				//. Get target information by reading directly from the target
	.auto_probe 		= pac52xx_auto_probe,			//.
	.erase_check 		= default_flash_blank_check,	//. Check to see if flash is empty
	.protect_check 		= pac52xx_protect_check,		//. Check to see if all sectors are unlocked
    .free_driver_priv = default_flash_free_driver_priv,
};
