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


#define PAC55XX_MEMCTL              0x400D0000
#define PAC55XX_MEMSTATUS           0x400D0004
#define PAC55XX_MEMCTL_FLASHLOCK    0x400D0008
#define PAC55XX_MEMCTL_FLASHPAGE    0x400D000C
#define PAC55XX_MEMCTL_FLASHERASE   0x400D0020
#define PAC55XX_CCSCTL	            0x400D0400
#define PAC55XX_CCSPLLCTL           0x400D0404

#define PAC55XX_FLASH_LOCK	        0x400D0008
#define PAC55XX_FLASH_PAGE	        0x400D000C
#define PAC55XX_FLASH_ERASE         0x400D0020

#define FLASH_LOCK_ALLOW_WRITE_MEMCTL       0xD513B490          // Write this value to FLASHLOCK to allow write to MEMCTL register
#define FLASH_LOCK_ALLOW_WRITE_ERASE_FLASH  0x43DF140A          // Write this value to FLASHLOCK to allow write and erase operations to FLASH
#define FLASH_LOCK_ALLOW_WRITE_SWDFUSE      0x79B4F762          // Write this value to FLASHLOCK to allow write access to INFO2.SWDFUSE to permanently disable SWD
#define FLASH_LOCK_ALLOW_WRITE_SECEN        0x1D855C1E          // Write this value to FLASHLOCK to allow writes to INFO2.SECEN

#define FLASH_START_PAGE_ERASE              0x8C799CA7          // Allow memory controller to start a FLASH page erase operation. 
#define FLASH_ERASE_INFO_3                  0x1266FF45          // Allow erase info-3 flash pages
#define FLASH_START_MASS_PAGE_ERASE         0x09EE76C9          // Start a Mass Erase of all flash memory pages
#define FLASH_START_MASS_PROG_INFO_ERASE    0x856E0E70          // Start a Mass Program and INFO3 Erase


#define PAC55XX_FLASH_BASE    	0x00000000UL
#define PAC55XX_NUM_FLASH_PAGES 128

#define FLASH_ERASE_TIMEOUT 100
#define FLASH_WRITE_TIMEOUT 5

// Data structures
struct pac55xx_flash_bank 
{
	int probed;
    uint32_t user_bank_size;
};


// Methods

static inline int pac55xx_get_flash_status(struct flash_bank *bank, uint32_t *status)
{
	struct target *target = bank->target;
 
	return target_read_u32( target, PAC55XX_MEMSTATUS, status ); 
}

static int pac55xx_wait_status_busy( struct flash_bank *bank, int timeout )
{
	uint32_t status;
	int retval = ERROR_OK;

	// Wait for the write or erase operation to complete or timeout
	for(;;) 
	{
		retval = pac55xx_get_flash_status( bank, &status );

		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("status: 0x%" PRIx32 "", status);

		// If EBUSY and WBUSY are both 0, operation is complete
		if( !(status & 0x3) )
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

static int pac55xx_init(struct flash_bank *bank)
{
	struct target *target = bank->target;
    int retval;
    uint32_t value;
    uint32_t pll_locked;
    
    // Initialize clocks
	// Set CCSCTL = 0x0000F105;   FRCLK=CLKREF, LDO=EN, SCLK=FRCLK, HCLKDIV=SCLK/1
    retval = target_write_u32( target, PAC55XX_CCSCTL, 0x0000F105 );
    if (retval != ERROR_OK)
        return retval;   
        
    // Configure PLL
    // Set CCSPLLCTL = 0x00012C45;  PLLCLK=150MHz, PLLEN=1, PLLBP=0, PLLOUTDIV=1 (/2), PLLINDIV=4, PLLFBDIV=300
    retval = target_write_u32( target, PAC55XX_CCSPLLCTL, 0x00012C45 );
    if (retval != ERROR_OK)
        return retval;   
    
    // Wait for PLL to be locked;   bit 24 of CCSPLLCTL is the locked bit
    do
    {
        target_read_u32( target, PAC55XX_CCSPLLCTL, &value );
        pll_locked = value & 0x01000000;
    }
    while (pll_locked == 0);                  /* Wait for PLL Lock */
 
    // Switch SCLK to PLL Clock and set HCLK = SCLK/2
	// Set CCSCTL = 0x0100F115;   FRCLK=CLKREF, LDO=EN, SCLK=PLLCLK, HCLKDIV=SCLK/2
    retval = target_write_u32( target, PAC55XX_CCSCTL, 0x0100F115 );
    if (retval != ERROR_OK)
        return retval; 
        
    // Set FLASHLOCK to allow access to MEMCTL reg
    retval = target_write_u32( target, PAC55XX_FLASH_LOCK, 0xD513B490 );
    if (retval != ERROR_OK)
        return retval; 
        
    // Set MCLK=30 MHz;  Set MEMCTL so MCLKSEL=MCLK and MCLKDIV = HCLK/5
    retval = target_write_u32( target, PAC55XX_MEMCTL, 0x00720046 );
    if (retval != ERROR_OK)
        return retval;     

    // Lock MEMCTL/FLASH again
    retval = target_write_u32( target, PAC55XX_FLASH_LOCK, 0x00000000 );
    if (retval != ERROR_OK)
        return retval; 
            
    return ERROR_OK;
}


static int pac55xx_erase(struct flash_bank *bank, int first, int last)
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
		// Allow Flash to be erased
		retval = target_write_u32( target, PAC55XX_MEMCTL_FLASHLOCK, FLASH_LOCK_ALLOW_WRITE_ERASE_FLASH );
		if (retval != ERROR_OK)
			return retval;

        // Set Page to Be Written
        //PAC55XX_MEMCTL->FLASHPAGE.PAGE = page_num;
		retval = target_write_u32( target, PAC55XX_MEMCTL_FLASHPAGE, i );
		if (retval != ERROR_OK)
			return retval;

		// Start page erase
		retval = target_write_u32( target, PAC55XX_MEMCTL_FLASHERASE, FLASH_START_PAGE_ERASE );
		if (retval != ERROR_OK)
			return retval;
		
		// Wait for erase to complete or time out
		retval = pac55xx_wait_status_busy( bank, FLASH_ERASE_TIMEOUT );
		if (retval != ERROR_OK)
			return retval;

		bank->sectors[i].is_erased = 1;
	}

    // Disable Flash Erase access
	retval = target_write_u32( target, PAC55XX_MEMCTL_FLASHLOCK, 0 );
	if (retval != ERROR_OK)
		return retval;


	return ERROR_OK;
}


static int pac55xx_erase_key(struct flash_bank *bank, uint32_t key)
{
	int retval;
	struct target *target = bank->target;
	//struct pac55xx_flash_bank *pac55xx_info = NULL;
    volatile uint32_t i;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

    // Not currently used
	//pac55xx_info = bank->driver_priv;

    // Allow Flash to be erased
    retval = target_write_u32( target, PAC55XX_MEMCTL_FLASHLOCK, FLASH_LOCK_ALLOW_WRITE_ERASE_FLASH );
    if (retval != ERROR_OK)
        return retval;

    // Start erase with specified key
    retval = target_write_u32( target, PAC55XX_MEMCTL_FLASHERASE, key );
    if (retval != ERROR_OK)
        return retval;
    
    // Wait for erase to complete or time out
    retval = pac55xx_wait_status_busy( bank, FLASH_ERASE_TIMEOUT );
    if (retval != ERROR_OK)
        return retval;

    for (i = 0; i < PAC55XX_NUM_FLASH_PAGES; i++)
    {
        bank->sectors[i].is_erased = 1;
    }


    // Disable Flash Erase access
	retval = target_write_u32( target, PAC55XX_MEMCTL_FLASHLOCK, 0 );
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}


static int pac55xx_protect(struct flash_bank *bank, int set, int first, int last)
{
	LOG_DEBUG("PROTECT NOT YET IMPLEMENTED");

	return ERROR_OK;
}


static int pac55xx_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
    // The memory controller requires all flash writes to start on a 16-byte boundary and consist of multiples of 16 bytes in size
    // If the desired amount to be written is less than 16 bytes, this code writes the other bytes as 0xFF to preserve the contents.
    // Note that reads or fetches from Flash should not take place until WBUSY=0 and an additional delay of 10 uSec has been added

	struct target *target = bank->target;
    int retval;
    uint32_t size_bytes = count;
    const uint8_t *p_src = (const uint8_t *)buffer;
    union {
        uint8_t b[16];
        uint32_t w[4];
    }buff;
    uint8_t i;
    uint32_t src_index;
    uint8_t buff_index;

    
	// Make sure target is halted
	if( bank->target->state != TARGET_HALTED ) 
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

    //===== Clear WRITEWORDCNT in case it's not 0; must set FLASHLOCK to allow write to MEMCTL =====
    // Set FLASHLOCK to allow access to MEMCTL reg
    retval = target_write_u32( target, PAC55XX_FLASH_LOCK, 0xD513B490 );
    if (retval != ERROR_OK)
        return retval; 
    // PAC55XX_MEMCTL->MEMCTL.WRITEWORDCNT(bit 9:8) = 0;    Rest of MEMCTL remains the same
    retval = target_write_u32( target, PAC55XX_MEMCTL, 0x00720046 );
    if (retval != ERROR_OK)
        return retval; 
    
    
    // Set FLASHLOCK to allow Writes to Flash
    retval = target_write_u32( target, PAC55XX_MEMCTL_FLASHLOCK, FLASH_LOCK_ALLOW_WRITE_ERASE_FLASH );
    if (retval != ERROR_OK)
        return retval;

    // Set address for Flash writes to 16 byte boundary;  
    // All flash writes must start on a 16-byte boundary and consist of 16 bytes in size
    uint32_t address = offset & ~0xF;

    // buff_index points to the first byte to contain valid data
    buff_index = offset & 0xF;

    // Fill 16 byte temp buffer with FFs up to buff_index (must write FFs to bytes that shouldn't change)
    for(i=0; i < buff_index; i++)
    {
        buff.b[i] = 0xFF;
    }

    src_index = 0;
    // Fill the temp buffer and write when it's full
    while(size_bytes--)
    {
        // Copy enough bytes from source buffer into temp buffer to reach 16 bytes
        buff.b[buff_index++] = p_src[src_index++];

        // If we have 16 bytes in temp buff, then write the 16 bytes of data
        if(buff_index == 16)
        {
            // Write 4 words (16 bytes) to Flash
            for(i=0; i < 4; i++)
            {
                target_write_u32(target, address, buff.w[i] );                
                address += sizeof(uint32_t);
            }

            // Wait for Write to complete or time out
            retval = pac55xx_wait_status_busy( bank, FLASH_WRITE_TIMEOUT );
            if (retval != ERROR_OK)
                return retval;
            
            buff_index = 0;                     // Reset buff_index for next time through
        }
    }

    // If index is non zero then we need to fill remaining buffer with FFs and write last bytes
    if(buff_index)
    {
        // Fill remainder of buffer with FFs
        for(i=buff_index; i < 16; i++)
        {
             buff.b[i] = 0xFF;
        }

        // Write final 4 32-bit words (16 bytes) to Flash
        for(i=0; i < 4; i++)
        {
            target_write_u32( target, address, buff.w[i] );
            address += sizeof(uint32_t);
        }
        // Wait for Write to complete or time out
        retval = pac55xx_wait_status_busy( bank, FLASH_WRITE_TIMEOUT );
        if (retval != ERROR_OK)
            return retval;        
        
    }

    //sleep 1 ms; following last write to flash, need to wait at least an additional 10us after WBUSY=0 before flash can be read
    alive_sleep(1);

    // Return FLASHLOCK to locked state
    // PAC55XX_MEMCTL->FLASHLOCK =0;
    retval = target_write_u32( target, PAC55XX_MEMCTL_FLASHLOCK, 0 );
    if (retval != ERROR_OK)
    {
        return retval;
    }
    
	return ERROR_OK;
}


static int pac55xx_probe( struct flash_bank *bank )
{
	struct pac55xx_flash_bank *pac55xx_info = bank->driver_priv;

	int i;

	// Set base flash address
	uint32_t base_address = PAC55XX_FLASH_BASE;

	// Set bank info
	pac55xx_info->probed = 0;

	// 128 1Kb pages
	int num_pages = 128;
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

    // Initialize pac55xx
    pac55xx_init(bank);
    
	// Mark bank as probed
	pac55xx_info->probed = 1;

	return ERROR_OK;
}

static int pac55xx_auto_probe(struct flash_bank *bank)
{
	struct pac55xx_flash_bank *pac55xx_info = bank->driver_priv;

	if (pac55xx_info->probed)
	{
		// Already probed
		return ERROR_OK;
	}
	else
	{
		// Execute probe
		return pac55xx_probe( bank );
	}
}

static int pac55xx_protect_check(struct flash_bank *bank)
{
	// TODO: Read write protection status of all sectors
	LOG_DEBUG("PROTECT_CHECK NOT YET IMPLEMENTED");

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(pac55xx_flash_bank_command)
{
	struct pac55xx_flash_bank *pac55xx_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	pac55xx_info = malloc(sizeof(struct pac55xx_flash_bank));

	bank->driver_priv = pac55xx_info;
	pac55xx_info->probed = 0;
    
    pac55xx_info->user_bank_size = bank->size;

	return ERROR_OK;
}

COMMAND_HANDLER(pac55xx_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1) {
		command_print(CMD, "pac55xx mass_erase <bank>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = pac55xx_erase_key(bank, FLASH_START_MASS_PROG_INFO_ERASE);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD, "pac55xx mass erase complete");
	} else {
		command_print(CMD, "pac55xx mass erase failed");
	}

	return retval;
}

COMMAND_HANDLER(pac55xx_handle_info_command)
{
	LOG_DEBUG("INFO NOT YET IMPLEMENTED");
	return ERROR_OK;
}

static const struct command_registration pac55xx_exec_command_handlers[] = 
{
	{
		.name = "info",
		.handler = pac55xx_handle_info_command,
		.mode = COMMAND_EXEC,
		.usage = "N/A",
		.help = "Get chip info"
	},
	{
		.name = "mass_erase",
		.handler = pac55xx_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire device flash.",
	},
    
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration pac55xx_command_handlers[] = 
{
	{
		.name = "pac55xx",
		.mode = COMMAND_ANY,
		.help = "PAC55xx flash command group",
		.usage = "",
		.chain = pac55xx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver pac55xx_flash = 
{
	.name 				= "pac55xx",
	.commands 			= pac55xx_command_handlers,		//. Additional chip specific commands
	.flash_bank_command	= pac55xx_flash_bank_command,	//. Flash bank setup
	.erase 				= pac55xx_erase,				//. Erase sectors
	.protect 			= pac55xx_protect,				//. Enable or disable protection for sectors
	.write 				= pac55xx_write,				// Write X bytes to offset position
	.read 				= default_flash_read,			//. Read X bytes from offset position
	.probe 				= pac55xx_probe,				//. Get target information by reading directly from the target
	.auto_probe 		= pac55xx_auto_probe,			//.
	.erase_check 		= default_flash_blank_check,	//. Check to see if flash is empty
	.protect_check 		= pac55xx_protect_check,		//. Check to see if all sectors are unlocked
    .free_driver_priv = default_flash_free_driver_priv,
};

