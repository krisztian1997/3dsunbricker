/** 
  * Copyright (c) by Roland Riegel, Ryuga, Krisztian      
  *
  **/
#include <string.h>
#include <avr/io.h>
#include <sd_raw_roland.h>
#include "Arduino.h"
#include "crc.c"
/**
 * \addtogroup sd_raw MMC/SD/SDHC card raw access
 *
 * This module implements read and write access to MMC, SD
 * and SDHC cards. It serves as a low-level driver for the
 * higher level modules such as partition and file system
 * access.
 *
 * @{
 */
/**
 * \file
 * MMC/SD/SDHC raw access implementation (license: GPLv2 or LGPLv2.1)
 *
 * \author Roland Riegel
 */
 
/**
 * \file
 * SPI unlock/lock/force erase implementation (license: GPLv2 or LGPLv2.1)
 *
 * \author Krisztian, Ryuga with some help from bkifft
 */
 
/**
 * \addtogroup sd_raw_config MMC/SD configuration
 * Preprocessor defines to configure the MMC/SD support.
 */

/**
 * @}
 */

/* commands available in SPI mode */

/* CMD0: response R1 */
#define CMD_GO_IDLE_STATE 0x00
/* CMD1: response R1 */
#define CMD_SEND_OP_COND 0x01
/* CMD8: response R7 */
#define CMD_SEND_IF_COND 0x08
/* CMD9: response R1 */
#define CMD_SEND_CSD 0x09
/* CMD10: response R1 */
#define CMD_SEND_CID 0x0a
/* CMD12: response R1 */
#define CMD_STOP_TRANSMISSION 0x0c
/* CMD13: response R2 */
#define CMD_SEND_STATUS 0x0d
/* CMD16: arg0[31:0]: block length, response R1 */
#define CMD_SET_BLOCKLEN 0x10
/* CMD17: arg0[31:0]: data address, response R1 */
#define CMD_READ_SINGLE_BLOCK 0x11
/* CMD18: arg0[31:0]: data address, response R1 */
#define CMD_READ_MULTIPLE_BLOCK 0x12
/* CMD24: arg0[31:0]: data address, response R1 */
#define CMD_WRITE_SINGLE_BLOCK 0x18
/* CMD25: arg0[31:0]: data address, response R1 */
#define CMD_WRITE_MULTIPLE_BLOCK 0x19
/* CMD27: response R1 */
#define CMD_PROGRAM_CSD 0x1b
/* CMD28: arg0[31:0]: data address, response R1b */
#define CMD_SET_WRITE_PROT 0x1c
/* CMD29: arg0[31:0]: data address, response R1b */
#define CMD_CLR_WRITE_PROT 0x1d
/* CMD30: arg0[31:0]: write protect data address, response R1 */
#define CMD_SEND_WRITE_PROT 0x1e
/* CMD32: arg0[31:0]: data address, response R1 */
#define CMD_TAG_SECTOR_START 0x20
/* CMD33: arg0[31:0]: data address, response R1 */
#define CMD_TAG_SECTOR_END 0x21
/* CMD34: arg0[31:0]: data address, response R1 */
#define CMD_UNTAG_SECTOR 0x22
/* CMD35: arg0[31:0]: data address, response R1 */
#define CMD_TAG_ERASE_GROUP_START 0x23
/* CMD36: arg0[31:0]: data address, response R1 */
#define CMD_TAG_ERASE_GROUP_END 0x24
/* CMD37: arg0[31:0]: data address, response R1 */
#define CMD_UNTAG_ERASE_GROUP 0x25
/* CMD38: arg0[31:0]: stuff bits, response R1b */
#define CMD_ERASE 0x26
/* ACMD41: arg0[31:0]: OCR contents, response R1 */
#define CMD_SD_SEND_OP_COND 0x29
/* CMD42: arg0[31:0]: stuff bits, response R1b */
#define CMD_LOCK_UNLOCK 0x2a
/* CMD55: arg0[31:0]: stuff bits, response R1 */
#define CMD_APP 0x37
/* CMD58: arg0[31:0]: stuff bits, response R3 */
#define CMD_READ_OCR 0x3a
/* CMD59: arg0[31:1]: stuff bits, arg0[0:0]: crc option, response R1 */
#define CMD_CRC_ON_OFF 0x3b

/* command responses */
/* R1: size 1 byte */
#define R1_IDLE_STATE 0
#define R1_ERASE_RESET 1
#define R1_ILL_COMMAND 2
#define R1_COM_CRC_ERR 3
#define R1_ERASE_SEQ_ERR 4
#define R1_ADDR_ERR 5
#define R1_PARAM_ERR 6
/* R1b: equals R1, additional busy bytes */
/* R2: size 2 bytes */
#define R2_CARD_LOCKED 0
#define R2_WP_ERASE_SKIP 1
#define R2_ERR 2
#define R2_CARD_ERR 3
#define R2_CARD_ECC_FAIL 4
#define R2_WP_VIOLATION 5
#define R2_INVAL_ERASE 6
#define R2_OUT_OF_RANGE 7
#define R2_CSD_OVERWRITE 7
#define R2_IDLE_STATE (R1_IDLE_STATE + 8)
#define R2_ERASE_RESET (R1_ERASE_RESET + 8)
#define R2_ILL_COMMAND (R1_ILL_COMMAND + 8)
#define R2_COM_CRC_ERR (R1_COM_CRC_ERR + 8)
#define R2_ERASE_SEQ_ERR (R1_ERASE_SEQ_ERR + 8)
#define R2_ADDR_ERR (R1_ADDR_ERR + 8)
#define R2_PARAM_ERR (R1_PARAM_ERR + 8)
/* R3: size 5 bytes */
#define R3_OCR_MASK (0xffffffffUL)
#define R3_IDLE_STATE (R1_IDLE_STATE + 32)
#define R3_ERASE_RESET (R1_ERASE_RESET + 32)
#define R3_ILL_COMMAND (R1_ILL_COMMAND + 32)
#define R3_COM_CRC_ERR (R1_COM_CRC_ERR + 32)
#define R3_ERASE_SEQ_ERR (R1_ERASE_SEQ_ERR + 32)
#define R3_ADDR_ERR (R1_ADDR_ERR + 32)
#define R3_PARAM_ERR (R1_PARAM_ERR + 32)
/* Data Response: size 1 byte */
#define DR_STATUS_MASK 0x0e
#define DR_STATUS_ACCEPTED 0x05
#define DR_STATUS_CRC_ERR 0x0a
#define DR_STATUS_WRITE_ERR 0x0c

/* status bits for card types */
#define SD_RAW_SPEC_1 0
#define SD_RAW_SPEC_2 1
#define SD_RAW_SPEC_SDHC 2

#if !SD_RAW_SAVE_RAM
/* static data buffer for acceleration */
static uint8_t raw_block[512];
/* offset where the data within raw_block lies on the card */
static offset_t raw_block_address;
#if SD_RAW_WRITE_BUFFERING
/* flag to remember if raw_block was written to the card */
static uint8_t raw_block_written;
#endif
#endif



/* private helper functions */
static void 					sd_raw_send_byte(uint8_t b);
static uint8_t 					sd_raw_rec_byte();
static uint8_t 					sd_raw_send_command(uint8_t command, uint32_t arg);
static uint8_t 					sd_raw_send_command_crc(uint8_t command, uint32_t arg);
static void 					ReadCardStatus();
static void 					ShowCardStatus();
static void  					LoadGlobalPWD(void);
static int8_t  					sd_wait_for_data();
static unsigned char  			xchg(unsigned char c);
static uint8_t 					send_cmd42_erase();
static uint8_t 					erase();
static uint8_t 					pwd_lock();
static uint8_t 					pwd_unlock();
static uint8_t 					sd_raw_send_reset();
static void 					menu();
static int8_t  					ReadCID(void);
static void 					unlock_XOR();
static void						lock_XOR();
static void 					dedication();
static inline 					uint32_t byte_swap(uint32_t in);
void 							PrintHex8(uint8_t *data, uint8_t length);
static void 					dedication();
static void                                     GenerateCRCTable();
static uint8_t                                  AddByteToCRC(uint8_t  crc, uint8_t  b);
static uint8_t                                  ReadCSD();
static uint8_t                                  SetTmpWriteProc();
static uint8_t                                  ResetTmpWriteProc();

#define  CRC7_POLY              0x89
/* Variable declarations */
static uint8_t sd_raw_card_type; // card type state
uint8_t cardstatus[2]; // card status array to store a R2 response
uint8_t pwd[17]; 
uint8_t pwd_len;
uint8_t crctable[256];
uint8_t csd[16];
char GlobalPWDStr[16] = {'T', 'W', 'I', 'L', 'I', 'G', 'S', 'P', 'O', 'R', 'K', 'L', 'E', 'P', 'A', 'H'}; // password used to lock the card
unsigned char mess[12] ={0x15, 0x47, 0xC3, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF}; // dont touch this array, its used by the CRC16
uint8_t terminateExecution = 0;
#define  GLOBAL_PWD_LEN      (sizeof(GlobalPWDStr))
uint8_t ignited = 0;

void sd_raw_configure(){
	/* enable inputs for reading card status */
    configure_pin_available();
    configure_pin_locked();

    /* enable outputs for MOSI, SCK, SS, input for MISO */
    configure_pin_mosi();
    configure_pin_sck();
    configure_pin_ss();
    configure_pin_miso();
    Serial.print(F("Enabled outputs/inputs"));
    unselect_card();

    /* initialize SPI with lowest frequency; max. 400kHz during identification mode of card */
    SPCR = (0 << SPIE) | /* SPI Interrupt Enable */
           (1 << SPE)  | /* SPI Enable */
           (0 << DORD) | /* Data Order: MSB first */
           (1 << MSTR) | /* Master mode */
           (0 << CPOL) | /* Clock Polarity: SCK low when idle */
           (0 << CPHA) | /* Clock Phase: sample on rising SCK edge */
           (1 << SPR1) | /* Clock Frequency: f_OSC / 128 */
           (0 << SPR0);
    SPSR &= ~(1 << SPI2X); /* No doubled clock frequency */
    Serial.print("\nInitialized SPI with 250khz frequency");	
}

/**
 * \ingroup sd_raw
 * Initializes memory card communication.
 *
 * \returns 0 on failure, 1 on success.
 */
uint8_t sd_raw_init()
{	
	if(!ignited){ sd_raw_configure(); /*ignited = 1;*/} else {}
	uint8_t eraser = 0; //set this to 1 if you want to erase the card, 0 if you want to set the password, 2 if you want to unlock
    
    /* initialization procedure */
    sd_raw_card_type = 0;

    if(!sd_raw_available())
        return 0;
    Serial.print(F("\nWaiting the minimum 80 cycles for warm up"));
    /* card needs 74 cycles minimum to start up */
    for(uint8_t i = 0; i < 12; ++i)
    {
        /* wait 8 clock cycles */
        sd_raw_rec_byte();
    }
    /* address card */
    select_card();
    Serial.print(F("\nKeeping CS line on low for communication"));
    /* reset card */
    uint8_t response;
    for(uint16_t i = 0; ; ++i)
    {
        response = sd_raw_send_reset();
        if(response == (1 << R1_IDLE_STATE))
            break;

        if(i == 0x1ff)
        {
            unselect_card();
            Serial.print(F("\nReset command sent on MOSI, but no answer from the slave. Please check your card connection/soldering"));
            return 0;

        }
    }
    Serial.print(F("\nCard is in IDLE. Checking if the voltage is correct."));
#if SD_RAW_SDHC
    /* check for version of SD card specification */
    response = sd_raw_send_command(CMD_SEND_IF_COND, 0x100 /* 2.7V - 3.6V */ | 0xaa /* test pattern */);
    if((response & (1 << R1_ILL_COMMAND)) == 0)
    {
        sd_raw_rec_byte();
        sd_raw_rec_byte();
        if((sd_raw_rec_byte() & 0x01) == 0)
            return 0; /* card operation voltage range doesn't match */
        if(sd_raw_rec_byte() != 0xaa)
            return 0; /* wrong test pattern */

        /* card conforms to SD 2 card specification */
        sd_raw_card_type |= (1 << SD_RAW_SPEC_2);
        Serial.print(F("\nThis is an SDHC (SD2) card"));
    }
    else
#endif
    {
        /* determine SD/MMC card type */
        sd_raw_send_command(CMD_APP, 0);
        response = sd_raw_send_command(CMD_SD_SEND_OP_COND, 0);
        if((response & (1 << R1_ILL_COMMAND)) == 0)
        {
            /* card conforms to SD 1 card specification */
            sd_raw_card_type |= (1 << SD_RAW_SPEC_1);
            Serial.print(F("\nThis is an SD (SD1) card"));
        }
        else
        {
            /* MMC card */
            Serial.print(F("\nMMC card"));
        }
    }

    /* wait for card to get ready */
    for(uint16_t i = 0; ; ++i)
    {
        if(sd_raw_card_type & ((1 << SD_RAW_SPEC_1) | (1 << SD_RAW_SPEC_2)))
        {
            uint32_t arg = 0;
#if SD_RAW_SDHC
            if(sd_raw_card_type & (1 << SD_RAW_SPEC_2))
                arg = 0x40000000;
#endif
            sd_raw_send_command(CMD_APP, 0);
            response = sd_raw_send_command(CMD_SD_SEND_OP_COND, arg);
        }
        else
        {
            response = sd_raw_send_command(CMD_SEND_OP_COND, 0);
        }

        if((response & (1 << R1_IDLE_STATE)) == 0)
            break;

        if(i == 0x7fff)
        {
            unselect_card();
            Serial.print(F("\nCard is not ready for some reasons, stopping communication with the slave"));
            return 0;
        }
    }

#if SD_RAW_SDHC
    if(sd_raw_card_type & (1 << SD_RAW_SPEC_2))
    {
        if(sd_raw_send_command(CMD_READ_OCR, 0))
        {
            unselect_card();
            return 0;
        }

        if(sd_raw_rec_byte() & 0x40)
            sd_raw_card_type |= (1 << SD_RAW_SPEC_SDHC);

        sd_raw_rec_byte();
        sd_raw_rec_byte();
        sd_raw_rec_byte();
    }
#endif

    /* set block size to 512 bytes */
    if(sd_raw_send_command(CMD_SET_BLOCKLEN, 512))
    {
        Serial.print(F("\nCan't set BLOCKLEN to 512 bytes"));
        unselect_card();
        return 0;
    }else{
		Serial.print(F("\nSet SET_BLOCKLEN to 512 byte"));
	}
    ShowCardStatus();
    /* deaddress card */
    unselect_card();
    /* erase procedure, comment this if you dont want to erase your card */
    Serial.println();
	if(Serial){
		select_card();
		menu();
	}else{
		if(eraser == 0){
			Serial.println(erase(), BIN);
			Serial.print(F("\nAutomatically erased the card"));
		}else if(eraser == 1){
			Serial.println(pwd_lock(), BIN);
			Serial.print(F("\nAutomatically locked the card"));
		}else if(eraser == 2){
			Serial.println(pwd_unlock(), BIN);
			Serial.print(F("\nAutomatically unlocked the card"));
		}
	}
    ShowCardStatus();
    Serial.println();
    /* switch to highest SPI frequency possible */
    SPCR &= ~((1 << SPR1) | (1 << SPR0)); /* Clock Frequency: f_OSC / 4 */
    SPSR |= (1 << SPI2X); /* Doubled Clock Frequency: f_OSC / 2 */
    Serial.print("\nSwitched to highest SPI frequency");
    return 1;
}

/**
 * \ingroup sd_raw
 * Checks wether a memory card is located in the slot.
 *
 * \returns 1 if the card is available, 0 if it is not.
 */
uint8_t sd_raw_available()
{
    return get_pin_available() == 0x00;
}

/**
 * \ingroup sd_raw
 * Checks wether the memory card is locked for write access.
 *
 * \returns 1 if the card is locked, 0 if it is not.
 */
uint8_t sd_raw_locked()
{
    return get_pin_locked() == 0x00;
}

/**
 * \ingroup sd_raw
 * Sends a raw byte to the memory card.
 *
 * \param[in] b The byte to sent.
 * \see sd_raw_rec_byte
 */
void sd_raw_send_byte(uint8_t b)
{
    SPDR = b;
    /* wait for byte to be shifted out */
    while(!(SPSR & (1 << SPIF)));
    SPSR &= ~(1 << SPIF);
}

/**
 * \ingroup sd_raw
 * Receives a raw byte from the memory card.
 *
 * \returns The byte which should be read.
 * \see sd_raw_send_byte
 */
uint8_t sd_raw_rec_byte()
{
    /* send dummy data for receiving some */
    SPDR = 0xff;
    while(!(SPSR & (1 << SPIF)));
    SPSR &= ~(1 << SPIF);

    return SPDR;
}

/**
 * \ingroup sd_raw
 * Send a command to the memory card which responses with a R1 response (and possibly others).
 *
 * \param[in] command The command to send.
 * \param[in] arg The argument for command.
 * \returns The command answer.
 */
uint8_t sd_raw_send_command(uint8_t command, uint32_t arg)
{
    uint8_t response;

    /* wait some clock cycles */
    sd_raw_rec_byte();

    /* send command via SPI */
    sd_raw_send_byte(0x40 | command);
    sd_raw_send_byte((arg >> 24) & 0xff);
    sd_raw_send_byte((arg >> 16) & 0xff);
    sd_raw_send_byte((arg >> 8) & 0xff);
    sd_raw_send_byte((arg >> 0) & 0xff);
    switch(command)
    {
    case CMD_GO_IDLE_STATE:
        sd_raw_send_byte(0x95);
        break;
    case CMD_SEND_IF_COND:
        sd_raw_send_byte(0x87);
        break;
    default:
        sd_raw_send_byte(0xff);
        break;
    }

    /* receive response */
    for(uint8_t i = 0; i < 10; ++i)
    {
        response = sd_raw_rec_byte();
        if(response != 0xff)
            break;
    }

    return response;
}

/**
 * \ingroup sd_raw
 * Reads raw data from the card.
 *
 * \param[in] offset The offset from which to read.
 * \param[out] buffer The buffer into which to write the data.
 * \param[in] length The number of bytes to read.
 * \returns 0 on failure, 1 on success.
 * \see sd_raw_read_interval, sd_raw_write, sd_raw_write_interval
 */
uint8_t sd_raw_read(offset_t offset, uint8_t* buffer, uintptr_t length)
{
    offset_t block_address;
    uint16_t block_offset;
    uint16_t read_length;
    while(length > 0)
    {
        /* determine byte count to read at once */
        block_offset = offset & 0x01ff;
        block_address = offset - block_offset;
        read_length = 512 - block_offset; /* read up to block border */
        if(read_length > length)
            read_length = length;

#if !SD_RAW_SAVE_RAM
        /* check if the requested data is cached */
        if(block_address != raw_block_address)
#endif
        {
#if SD_RAW_WRITE_BUFFERING
            if(!sd_raw_sync())
                return 0;
#endif

            /* address card */
            select_card();

            /* send single block request */
#if SD_RAW_SDHC
            if(sd_raw_send_command(CMD_READ_SINGLE_BLOCK, (sd_raw_card_type & (1 << SD_RAW_SPEC_SDHC) ? block_address / 512 : block_address)))
#else
            if(sd_raw_send_command(CMD_READ_SINGLE_BLOCK, block_address))
#endif
            {
                unselect_card();
                return 0;
            }

            /* wait for data block (start byte 0xfe) */
            while(sd_raw_rec_byte() != 0xfe);

#if SD_RAW_SAVE_RAM
            /* read byte block */
            uint16_t read_to = block_offset + read_length;
            for(uint16_t i = 0; i < 512; ++i)
            {
                uint8_t b = sd_raw_rec_byte();
                if(i >= block_offset && i < read_to)
                    *buffer++ = b;
            }
#else
            /* read byte block */
            uint8_t* cache = raw_block;
            for(uint16_t i = 0; i < 512; ++i)
                *cache++ = sd_raw_rec_byte();
            raw_block_address = block_address;

            memcpy(buffer, raw_block + block_offset, read_length);
            buffer += read_length;
#endif

            /* read crc16 */
            sd_raw_rec_byte();
            sd_raw_rec_byte();

            /* deaddress card */
            unselect_card();

            /* let card some time to finish */
            sd_raw_rec_byte();
        }
#if !SD_RAW_SAVE_RAM
        else
        {
            /* use cached data */
            memcpy(buffer, raw_block + block_offset, read_length);
            buffer += read_length;
        }
#endif

        length -= read_length;
        offset += read_length;
    }

    return 1;
}

/**
 * \ingroup sd_raw
 * Continuously reads units of \c interval bytes and calls a callback function.
 *
 * This function starts reading at the specified offset. Every \c interval bytes,
 * it calls the callback function with the associated data buffer.
 *
 * By returning zero, the callback may stop reading.
 *
 * \note Within the callback function, you can not start another read or
 *       write operation.
 * \note This function only works if the following conditions are met:
 *       - (offset - (offset % 512)) % interval == 0
 *       - length % interval == 0
 *
 * \param[in] offset Offset from which to start reading.
 * \param[in] buffer Pointer to a buffer which is at least interval bytes in size.
 * \param[in] interval Number of bytes to read before calling the callback function.
 * \param[in] length Number of bytes to read altogether.
 * \param[in] callback The function to call every interval bytes.
 * \param[in] p An opaque pointer directly passed to the callback function.
 * \returns 0 on failure, 1 on success
 * \see sd_raw_write_interval, sd_raw_read, sd_raw_write
 */
uint8_t sd_raw_read_interval(offset_t offset, uint8_t* buffer, uintptr_t interval, uintptr_t length, sd_raw_read_interval_handler_t callback, void* p)
{
    if(!buffer || interval == 0 || length < interval || !callback)
        return 0;

#if !SD_RAW_SAVE_RAM
    while(length >= interval)
    {
        /* as reading is now buffered, we directly
         * hand over the request to sd_raw_read()
         */
        if(!sd_raw_read(offset, buffer, interval))
            return 0;
        if(!callback(buffer, offset, p))
            break;
        offset += interval;
        length -= interval;
    }

    return 1;
#else
    /* address card */
    select_card();

    uint16_t block_offset;
    uint16_t read_length;
    uint8_t* buffer_cur;
    uint8_t finished = 0;
    do
    {
        /* determine byte count to read at once */
        block_offset = offset & 0x01ff;
        read_length = 512 - block_offset;

        /* send single block request */
#if SD_RAW_SDHC
        if(sd_raw_send_command(CMD_READ_SINGLE_BLOCK, (sd_raw_card_type & (1 << SD_RAW_SPEC_SDHC) ? offset / 512 : offset - block_offset)))
#else
        if(sd_raw_send_command(CMD_READ_SINGLE_BLOCK, offset - block_offset))
#endif
        {
            unselect_card();
            return 0;
        }

        /* wait for data block (start byte 0xfe) */
        while(sd_raw_rec_byte() != 0xfe);

        /* read up to the data of interest */
        for(uint16_t i = 0; i < block_offset; ++i)
            sd_raw_rec_byte();

        /* read interval bytes of data and execute the callback */
        do
        {
            if(read_length < interval || length < interval)
                break;

            buffer_cur = buffer;
            for(uint16_t i = 0; i < interval; ++i)
                *buffer_cur++ = sd_raw_rec_byte();

            if(!callback(buffer, offset + (512 - read_length), p))
            {
                finished = 1;
                break;
            }

            read_length -= interval;
            length -= interval;

        }
        while(read_length > 0 && length > 0);

        /* read rest of data block */
        while(read_length-- > 0)
            sd_raw_rec_byte();

        /* read crc16 */
        sd_raw_rec_byte();
        sd_raw_rec_byte();

        if(length < interval)
            break;

        offset = offset - block_offset + 512;

    }
    while(!finished);

    /* deaddress card */
    unselect_card();

    /* let card some time to finish */
    sd_raw_rec_byte();

    return 1;
#endif
}

#if DOXYGEN || SD_RAW_WRITE_SUPPORT
/**
 * \ingroup sd_raw
 * Writes raw data to the card.
 *
 * \note If write buffering is enabled, you might have to
 *       call sd_raw_sync() before disconnecting the card
 *       to ensure all remaining data has been written.
 *
 * \param[in] offset The offset where to start writing.
 * \param[in] buffer The buffer containing the data to be written.
 * \param[in] length The number of bytes to write.
 * \returns 0 on failure, 1 on success.
 * \see sd_raw_write_interval, sd_raw_read, sd_raw_read_interval
 */
uint8_t sd_raw_write(offset_t offset, const uint8_t* buffer, uintptr_t length)
{
    if(sd_raw_locked())
        return 0;

    offset_t block_address;
    uint16_t block_offset;
    uint16_t write_length;
    while(length > 0)
    {
        /* determine byte count to write at once */
        block_offset = offset & 0x01ff;
        block_address = offset - block_offset;
        write_length = 512 - block_offset; /* write up to block border */
        if(write_length > length)
            write_length = length;

        /* Merge the data to write with the content of the block.
         * Use the cached block if available.
         */
        if(block_address != raw_block_address)
        {
#if SD_RAW_WRITE_BUFFERING
            if(!sd_raw_sync())
                return 0;
#endif

            if(block_offset || write_length < 512)
            {
                if(!sd_raw_read(block_address, raw_block, sizeof(raw_block)))
                    return 0;
            }
            raw_block_address = block_address;
        }

        if(buffer != raw_block)
        {
            memcpy(raw_block + block_offset, buffer, write_length);

#if SD_RAW_WRITE_BUFFERING
            raw_block_written = 0;

            if(length == write_length)
                return 1;
#endif
        }

        /* address card */
        select_card();

        /* send single block request */
#if SD_RAW_SDHC
        if(sd_raw_send_command(CMD_WRITE_SINGLE_BLOCK, (sd_raw_card_type & (1 << SD_RAW_SPEC_SDHC) ? block_address / 512 : block_address)))
#else
        if(sd_raw_send_command(CMD_WRITE_SINGLE_BLOCK, block_address))
#endif
        {
            unselect_card();
            return 0;
        }

        /* send start byte */
        sd_raw_send_byte(0xfe);

        /* write byte block */
        uint8_t* cache = raw_block;
        for(uint16_t i = 0; i < 512; ++i)
            sd_raw_send_byte(*cache++);

        /* write dummy crc16 */
        sd_raw_send_byte(0xff);
        sd_raw_send_byte(0xff);

        /* wait while card is busy */
        while(sd_raw_rec_byte() != 0xff);
        sd_raw_rec_byte();

        /* deaddress card */
        unselect_card();

        buffer += write_length;
        offset += write_length;
        length -= write_length;

#if SD_RAW_WRITE_BUFFERING
        raw_block_written = 1;
#endif
    }

    return 1;
}
#endif

#if DOXYGEN || SD_RAW_WRITE_SUPPORT
/**
 * \ingroup sd_raw
 * Writes a continuous data stream obtained from a callback function.
 *
 * This function starts writing at the specified offset. To obtain the
 * next bytes to write, it calls the callback function. The callback fills the
 * provided data buffer and returns the number of bytes it has put into the buffer.
 *
 * By returning zero, the callback may stop writing.
 *
 * \param[in] offset Offset where to start writing.
 * \param[in] buffer Pointer to a buffer which is used for the callback function.
 * \param[in] length Number of bytes to write in total. May be zero for endless writes.
 * \param[in] callback The function used to obtain the bytes to write.
 * \param[in] p An opaque pointer directly passed to the callback function.
 * \returns 0 on failure, 1 on success
 * \see sd_raw_read_interval, sd_raw_write, sd_raw_read
 */
uint8_t sd_raw_write_interval(offset_t offset, uint8_t* buffer, uintptr_t length, sd_raw_write_interval_handler_t callback, void* p)
{
#if SD_RAW_SAVE_RAM
#error "SD_RAW_WRITE_SUPPORT is not supported together with SD_RAW_SAVE_RAM"
#endif

    if(!buffer || !callback)
        return 0;

    uint8_t endless = (length == 0);
    while(endless || length > 0)
    {
        uint16_t bytes_to_write = callback(buffer, offset, p);
        if(!bytes_to_write)
            break;
        if(!endless && bytes_to_write > length)
            return 0;

        /* as writing is always buffered, we directly
         * hand over the request to sd_raw_write()
         */
        if(!sd_raw_write(offset, buffer, bytes_to_write))
            return 0;

        offset += bytes_to_write;
        length -= bytes_to_write;
    }

    return 1;
}
#endif

#if DOXYGEN || SD_RAW_WRITE_SUPPORT
/**
 * \ingroup sd_raw
 * Writes the write buffer's content to the card.
 *
 * \note When write buffering is enabled, you should
 *       call this function before disconnecting the
 *       card to ensure all remaining data has been
 *       written.
 *
 * \returns 0 on failure, 1 on success.
 * \see sd_raw_write
 */
uint8_t sd_raw_sync()
{
#if SD_RAW_WRITE_BUFFERING
    if(raw_block_written)
        return 1;
    if(!sd_raw_write(raw_block_address, raw_block, sizeof(raw_block)))
        return 0;
    raw_block_written = 1;
#endif
    return 1;
}
#endif

/**
 * \ingroup sd_raw
 * Reads informational data from the card.
 *
 * This function reads and returns the card's registers
 * containing manufacturing and status information.
 *
 * \note: The information retrieved by this function is
 *        not required in any way to operate on the card,
 *        but it might be nice to display some of the data
 *        to the user.
 *
 * \param[in] info A pointer to the structure into which to save the information.
 * \returns 0 on failure, 1 on success.
 */
uint8_t sd_raw_get_info(struct sd_raw_info* info)
{
    if(!info || !sd_raw_available())
        return 0;

    memset(info, 0, sizeof(*info));

    select_card();

    /* read cid register */
    if(sd_raw_send_command(CMD_SEND_CID, 0))
    {
        unselect_card();
        return 0;
    }
    while(sd_raw_rec_byte() != 0xfe);
    for(uint8_t i = 0; i < 18; ++i)
    {
        uint8_t b = sd_raw_rec_byte();

        switch(i)
        {
        case 0:
            info->manufacturer = b;
            break;
        case 1:
        case 2:
            info->oem[i - 1] = b;
            break;
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
            info->product[i - 3] = b;
            break;
        case 8:
            info->revision = b;
            break;
        case 9:
        case 10:
        case 11:
        case 12:
            info->serial |= (uint32_t) b << ((12 - i) * 8);
            break;
        case 13:
            info->manufacturing_year = b << 4;
            break;
        case 14:
            info->manufacturing_year |= b >> 4;
            info->manufacturing_month = b & 0x0f;
            break;
        }
    }

    /* read csd register */
    uint8_t csd_read_bl_len = 0;
    uint8_t csd_c_size_mult = 0;
#if SD_RAW_SDHC
    uint16_t csd_c_size = 0;
#else
    uint32_t csd_c_size = 0;
#endif
    uint8_t csd_structure = 0;
    if(sd_raw_send_command(CMD_SEND_CSD, 0))
    {
        unselect_card();
        return 0;
    }
    while(sd_raw_rec_byte() != 0xfe);
    for(uint8_t i = 0; i < 18; ++i)
    {
        uint8_t b = sd_raw_rec_byte();

        if(i == 0)
        {
            csd_structure = b >> 6;
        }
        else if(i == 14)
        {
            if(b & 0x40)
                info->flag_copy = 1;
            if(b & 0x20)
                info->flag_write_protect = 1;
            if(b & 0x10)
                info->flag_write_protect_temp = 1;
            info->format = (b & 0x0c) >> 2;
        }
        else
        {
#if SD_RAW_SDHC
            if(csd_structure == 0x01)
            {
                switch(i)
                {
                case 7:
                    b &= 0x3f;
                case 8:
                case 9:
                    csd_c_size <<= 8;
                    csd_c_size |= b;
                    break;
                }
                if(i == 9)
                {
                    ++csd_c_size;
                    info->capacity = (offset_t) csd_c_size * 512 * 1024;
                }
            }
            else if(csd_structure == 0x00)
#endif
            {
                switch(i)
                {
                case 5:
                    csd_read_bl_len = b & 0x0f;
                    break;
                case 6:
                    csd_c_size = b & 0x03;
                    csd_c_size <<= 8;
                    break;
                case 7:
                    csd_c_size |= b;
                    csd_c_size <<= 2;
                    break;
                case 8:
                    csd_c_size |= b >> 6;
                    ++csd_c_size;
                    break;
                case 9:
                    csd_c_size_mult = b & 0x03;
                    csd_c_size_mult <<= 1;
                    break;
                case 10:
                    csd_c_size_mult |= b >> 7;

                    info->capacity = (uint32_t) csd_c_size << (csd_c_size_mult + csd_read_bl_len + 2);
                    break;
                }
            }
        }
    }

    unselect_card();

    return 1;
}

//******

uint8_t sd_raw_send_reset(){
	uint8_t response,i,r;
	uint8_t arg = 0x00;
	uint8_t command = CMD_GO_IDLE_STATE;
    uint16_t crc = calc_crc(mess,((command&arg)|command),CRC16STARTBIT);
    select_card(); // select SD card first
    r=sd_raw_send_command(CMD_GO_IDLE_STATE,0);
    xchg((crc >> 8) & 0xff);
    xchg((crc >> 0) & 0xff);
	return r;
}

/**
 * \ingroup sd_raw
 * Send a command followed by a CRC16 checksum to the memory card which responses with a R1 response (and possibly others).
 *
 * \param[in] command The command to send.
 * \param[in] arg The argument for command.
 * \returns The command answer.
 */
 
uint8_t sd_raw_send_command_crc(uint8_t command, uint32_t arg)
{
    uint8_t response,i;
	uint16_t crc = calc_crc(mess,((command&arg)|command),CRC16STARTBIT);
    /* wait some clock cycles */
    sd_raw_rec_byte();

    /* send command via SPI */
    sd_raw_send_byte(0x40 | command);
    sd_raw_send_byte((arg >> 24) & 0xff);
    sd_raw_send_byte((arg >> 16) & 0xff);
    sd_raw_send_byte((arg >> 8) & 0xff);
    sd_raw_send_byte((arg >> 0) & 0xff);
    /*CRC16 CCITT*/
    //CRC16 is calculated over data given/sent (block length*n_frames_sent)
    //SPI does not force CRC checking, but i'll enable it anyway
    //CRC16=size(command+argument) (38 bits)
    xchg((crc >> 8) & 0xff);
    xchg((crc >> 0) & 0xff);

    /* receive response */
    for(i = 0; i < 10; ++i)
    {
        response = sd_raw_rec_byte();
        if(response != 0xff)
            break;
    }

    return response;
}

/**
 * \ingroup sd_raw
 * Exchange some data with the card over the MOSI
 *
 * \returns N/A
 */
static  unsigned char  xchg(unsigned char  c)
{
    SPDR = c;
    while ((SPSR & (1<<SPIF)) == 0)  ;
    return  SPDR;
}

/**
 * \ingroup sd_raw
 * Send CMD_SEND_STATUS to the card and store the R2 reponse in an array
 *
 * \returns N/A
 */
static void ReadCardStatus()
{
    cardstatus[0] = sd_raw_send_command(CMD_SEND_STATUS, 0);
    cardstatus[1] = xchg(0xff);
    Serial.print("\nReadCardStatus = ");
    Serial.print(cardstatus[0], BIN);
    Serial.print(",");
    Serial.print(cardstatus[1], BIN);
    xchg(0xff);
}

/**
 * \ingroup sd_raw
 * Show if the card is locked or unlocked
 *
 * \returns N/A
 */
static void ShowCardStatus(void)
{
    ReadCardStatus();
    Serial.print("\nPassword status: ");
    if ((cardstatus[1] & 0x01) ==  0)  Serial.print("un");
    Serial.print("locked\n");
}

/**
 * \ingroup sd_raw
 * Sends some data to the card untill the DAT0 line becomes high again after a busy response.
 *
 * \returns N/A
 */
static int8_t sd_wait_for_data(void)
{
    int16_t i;
    uint8_t r;
    //select_card();
    for (i=0; i<100; i++)
    {
        r = xchg(0xff);
        if (r != 0xff)  break;
    }
    return  (int8_t) r;
}

/**
 * \ingroup sd_raw
 * Loads the password into an array and converts the caracters to binary.
 *
 * \returns N/A
 */
static void LoadGlobalPWD(void)
{
    uint8_t                         i;

    for (i=0; i<GLOBAL_PWD_LEN; i++)
    {
        pwd[i] = pgm_read_byte(&(GlobalPWDStr[i]));
    }
    pwd_len = GLOBAL_PWD_LEN;
}

/**
 * \ingroup sd_raw
 * Send erase command followed by the coresponding CRC16 checksum
 *
 * \returns uint8_t with the R1 response
 */
static uint8_t erase()
{
    uint8_t response,i,r;
    uint8_t arg = 0x08;
    uint8_t command = 0x2a;
    uint16_t crc = calc_crc(mess,((command&arg)|command),CRC16STARTBIT);
    sd_raw_rec_byte();
    Serial.println(F("\nStarting erase procedure"));
    select_card(); // select SD card first
    sd_raw_send_command(CMD_CRC_ON_OFF, 0);
    if(sd_raw_send_command(CMD_SET_BLOCKLEN, 1))
    {
        Serial.print(F("\nIMPOSIBLE TO SET_BLOCKLEN to 1 byte\n"));
        unselect_card();
        return 0;
    }
    else
    {
        Serial.print(F("\nSET_BLOCKLEN to 1 byte\n"));
    }
    r=sd_raw_send_command(CMD_LOCK_UNLOCK,0);
    Serial.println(r);
    sd_wait_for_data();
    xchg(0xfe);
    xchg(arg);
    xchg((crc >> 8) & 0xff);
    xchg((crc >> 0) & 0xff);
    sd_wait_for_data();
	return r;
}

/**
 * \ingroup sd_raw
 * Send the lock command to the card with the password define at the start of the code
 *
 * \returns uint8_t with the R1 response
 */
static uint8_t pwd_lock()
{
    LoadGlobalPWD();
    uint8_t response,r;
    uint16_t i;
    uint8_t arg = 0x01;
    uint8_t command = 0x2a;
    uint16_t crc = calc_crc(mess,((command&arg)|command),CRC16STARTBIT);
    sd_raw_rec_byte();
    Serial.print(F("\nStarting locking procedure"));
    select_card(); // select SD card first
    /*r=sd_raw_send_command(CMD_CRC_ON_OFF, 0);*/
    r=sd_raw_send_command(CMD_SET_BLOCKLEN, pwd_len+2);
    r=sd_raw_send_command(CMD_LOCK_UNLOCK,0);
    sd_wait_for_data();
	xchg(0xfe);
    xchg(arg);
    xchg(pwd_len);
    for (i=0; i<=pwd_len; i++)                          // need to send one full block for CMD42
    {
        if (i < pwd_len)
        {
            xchg(pwd[i]);                                   // send each byte via SPI
        }
    }
    xchg((crc >> 8) & 0xff);
    xchg((crc >> 0) & 0xff);
    r =  sd_wait_for_data();
	Serial.print(F("\nDone"));
	return r;
}
	
/**
 * \ingroup sd_raw
 * Send the unlock command to the card with the password define at the start of the code
 *
 * \returns uint8_t with the R1 response
 */
static uint8_t pwd_unlock()
{
	LoadGlobalPWD();
    uint8_t response,r;
    uint16_t i;
    uint8_t arg = 0x02;
    uint8_t command = 0x2a;
    uint16_t crc = calc_crc(mess,((command&arg)|command),CRC16STARTBIT);
    sd_raw_rec_byte();
    Serial.print(F("\nStarting unlocking procedure"));
    select_card(); // select SD card first
    /*r=sd_raw_send_command(CMD_CRC_ON_OFF, 0);*/
    r=sd_raw_send_command(CMD_SET_BLOCKLEN, pwd_len+2);
    r=sd_raw_send_command(CMD_LOCK_UNLOCK,0);
    sd_wait_for_data();
	xchg(0xfe);
    xchg(arg);
    xchg(pwd_len);
    for (i=0; i<=pwd_len; i++)                          // need to send one full block for CMD42
    {
        if (i < pwd_len)
        {
            xchg(pwd[i]);                                   // send each byte via SPI
        }
    }
    xchg((crc >> 8) & 0xff);
    xchg((crc >> 0) & 0xff);
    r =  sd_wait_for_data();
	Serial.print(F("\nDone"));
	return r;
}

/**
 * \ingroup sd_raw
 * Print the hex with leading zeros
 *
 * \returns N/A
 */
void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
        Serial.print("0x"); 
        for (int i=0; i<length; i++) { 
          if (data[i]<0x10) {Serial.print("0");} 
          Serial.print(data[i],HEX); 
          //Serial.print(" "); 
        }
}

/**
 * \ingroup sd_raw
 * Code stolen from bkifft, used to swap the bytes in some cases
 *
 * \returns N/A
 */
static inline uint32_t byte_swap (uint32_t in)
{
  uint32_t b0 = in & 0xff;
  uint32_t b1 = (in >> 8) & 0xff;
  uint32_t b2 = (in >> 16) & 0xff;
  uint32_t b3 = (in >> 24) & 0xff;
  uint32_t ret = (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
  return ret;
}

/**
 * \ingroup sd_raw
 * Very experimental unlock function based on CID and XORpad
 *
 * \returns N/A
 */
static void unlock_XOR(){
	// Thanks a lot bkifft for this code, and for helping me fix random and weird bugs in my code
	// Variables
	uint32_t 		key[4];
    uint32_t 		temp;
    uint8_t 		cid3ds[16];
	int8_t			response;
	uint8_t			r;
	bool			swap;
	uint8_t arg = 0x02;
    uint8_t command = 0x2a;
	//the Vernam cipher key: 17C6987E4401EDDE371AC56865FFB562. thanks to the anonymous donator!
    Serial.print("\nTrying to unlock the nand with the generated password");
	//XORpad: 17C6987E4401EDDE371AC56865FFB562 split into 4x4bytes
    key[0] = 0x17C6987E;
    key[1] = 0x4401EDDE;
    key[2] = 0x371AC568;
    key[3] = 0x65FFB562;
	
	//Get the CID from the SDcard/eMMC
	for (int i=0; i<16; i++)  cid3ds[i] = 0;
	response = sd_raw_send_command(CMD_SEND_CID, 0);
	response = sd_wait_for_data();
	if (response != (int8_t)0xfe)
	{
		//Serial.print("\nERROR: FAILED READING CID");
	}
	for (int i=0; i<16; i++)
	{
		cid3ds[i] = xchg(0xff);
	}
	xchg(0xff);							// send the crc
	sd_wait_for_data();
	uint16_t crc = calc_crc(mess,((command&arg)|command),CRC16STARTBIT);
    Serial.print("\nCID from 3ds stored in uint8_t array: ");
    PrintHex8((uint8_t*)(cid3ds), 16);
	Serial.print(F("\nDEBUG: key byteswap\n"));
    //Swap the bites in the XORpad because of the biteorder
    if(swap) for (int i = 0; i<=3; ++i) key[i] = byte_swap(key[i]);
    Serial.print(F("\nDEBUG: Swapped key: 0x"));
    for (int i=0; i<=3; ++i)  Serial.print(key[i], HEX);
    //print the swapped key and the CID to test if nothing went wrong with it
    Serial.print(F("\nDEBUG: CID before XOR and command injection: "));
	PrintHex8((uint8_t*)(cid3ds), 16);
    //XOR 32bit of the CID with the XORpad
    ((uint32_t*) cid3ds)[0] ^= key[0];
    ((uint32_t*) cid3ds)[1] ^= key[1];
    ((uint32_t*) cid3ds)[2] ^= key[2];
    ((uint32_t*) cid3ds)[3] ^= key[3];
    //construct the payload
    //((uint8_t*) cid3ds)[0] = 0b00000010; //clear password
	((uint8_t*) cid3ds)[0] = arg;
    ((uint8_t*) cid3ds)[1] = 14; //14 byte password
    Serial.print(F("\nDEBUG: unlock payload: "));
    //for (int i = 0; i <16; ++i) Serial.print(((uint8_t*)(cid3ds))[i], HEX);
	PrintHex8((uint8_t*)(cid3ds), 16);
	Serial.print(F("\nExchanging the payload and data"));
	r=sd_raw_send_command(CMD_SET_BLOCKLEN, 16);
    r=sd_raw_send_command(CMD_LOCK_UNLOCK,0);
	sd_wait_for_data();
	xchg(0xfe);
	for (int i = 0; i <16; ++i) xchg(((uint8_t*)(cid3ds))[i] & 0xff);
	xchg((crc >> 8) & 0xff);
    xchg((crc >> 0) & 0xff);
	Serial.print("\nResponse: ");
	Serial.print(r);
    //TODO: add function which send all this to the eMMC, first 2 bytes being the CMD and the argument followed by the payload, which is all combined in the cid3ds variable
}

static void lock_XOR(){
	// Thanks a lot bkifft for this code, and for helping me fix random and weird bugs in my code
	// Variables
	uint32_t 		key[4];
    uint32_t 		temp;
    uint8_t 		cid3ds[16];
	int8_t			response;
	uint8_t			r;
	bool			swap;
	uint8_t arg = 0x01;
    uint8_t command = 0x2a;
	//the Vernam cipher key: 17C6987E4401EDDE371AC56865FFB562. thanks to the anonymous donator!
    Serial.print("\nTrying to unlock the nand with the generated password");
	//XORpad: 17C6987E4401EDDE371AC56865FFB562 split into 4x4bytes
    key[0] = 0x17C6987E;
    key[1] = 0x4401EDDE;
    key[2] = 0x371AC568;
    key[3] = 0x65FFB562;
	
	//Get the CID from the SDcard/eMMC
	for (int i=0; i<16; i++)  cid3ds[i] = 0;
	response = sd_raw_send_command(CMD_SEND_CID, 0);
	response = sd_wait_for_data();
	if (response != (int8_t)0xfe)
	{
		//Serial.print("\nERROR: FAILED READING CID");
	}
	for (int i=0; i<16; i++)
	{
		cid3ds[i] = xchg(0xff);
	}
	xchg(0xff);							// send the crc
	sd_wait_for_data();
	uint16_t crc = calc_crc(mess,((command&arg)|command),CRC16STARTBIT);
    Serial.print("\nCID from 3ds stored in uint8_t array: ");
    PrintHex8((uint8_t*)(cid3ds), 16);
	Serial.print(F("\nDEBUG: key byteswap\n"));
    //Swap the bites in the XORpad because of the biteorder
    if(swap) for (int i = 0; i<=3; ++i) key[i] = byte_swap(key[i]);
    Serial.print(F("\nDEBUG: Swapped key: 0x"));
    for (int i=0; i<=3; ++i)  Serial.print(key[i], HEX);
    //print the swapped key and the CID to test if nothing went wrong with it
    Serial.print(F("\nDEBUG: CID before XOR and command injection: "));
	PrintHex8((uint8_t*)(cid3ds), 16);
    //XOR 32bit of the CID with the XORpad
    ((uint32_t*) cid3ds)[0] ^= key[0];
    ((uint32_t*) cid3ds)[1] ^= key[1];
    ((uint32_t*) cid3ds)[2] ^= key[2];
    ((uint32_t*) cid3ds)[3] ^= key[3];
    //construct the payload
    //((uint8_t*) cid3ds)[0] = 0b00000010; //clear password
	((uint8_t*) cid3ds)[0] = arg;
    ((uint8_t*) cid3ds)[1] = 14; //14 byte password
    Serial.print(F("\nDEBUG: unlock payload: "));
    //for (int i = 0; i <16; ++i) Serial.print(((uint8_t*)(cid3ds))[i], HEX);
	PrintHex8((uint8_t*)(cid3ds), 16);
	Serial.print(F("\nExchanging the payload and data"));
	r=sd_raw_send_command(CMD_SET_BLOCKLEN, 16);
    r=sd_raw_send_command(CMD_LOCK_UNLOCK,0);
	sd_wait_for_data();
	xchg(0xfe);
	for (int i = 0; i <16; ++i) xchg(((uint8_t*)(cid3ds))[i] & 0xff);
	xchg((crc >> 8) & 0xff);
    xchg((crc >> 0) & 0xff);
	Serial.print("\nResponse: ");
	Serial.print(r);
    //TODO: add function which send all this to the eMMC, first 2 bytes being the CMD and the argument followed by the payload, which is all combined in the cid3ds variable
}

//Temp Write protect
static uint8_t  SetTmpWriteProc()
{
        uint8_t                         response;
        uint8_t                         tcrc;
        uint16_t                        i;
        Serial.print(F("\nSetting Temp Write Protect Bit"));
        GenerateCRCTable();
        ReadCSD();
        select_card();
		sd_wait_for_data();
        csd[14] = csd[14] | 0x10;
        response = sd_raw_send_command(CMD_PROGRAM_CSD, 0);
		sd_wait_for_data();
        Serial.print(F("\nCMD_PROGRAM_CSD Response: "));
        Serial.print(response,HEX);
        if (response != 0)
        {
                Serial.print(F("\nRead/Write Failed"));
                //unselect_card();
        }
        else
        {
        sd_wait_for_data();
        xchg(0xfe);                                                     // send data token marking start of data block
 
        tcrc = 0;
        for (i=0; i<15; i++)                            // for all 15 data bytes in CSD...
        {
        xchg(csd[i]);                                   // send each byte via SPI
                tcrc = AddByteToCRC(tcrc, csd[i]);              // add byte to CRC
        }
        xchg((tcrc<<1) + 1);                            // format the CRC7 value and send it
 
        xchg(0xff);                                                     // ignore dummy checksum
        xchg(0xff);                                                     // ignore dummy checksum
 
        i = 0xffff;                                                     // max timeout
        while (!xchg(0xFF) && (--i))  ;         // wait until we are not busy
 
        if (i)  Serial.print(F("\nSet Temp Write Proctect Done"));                 // return success
        else  Serial.println(F("\nSet Temp Write Proctect Failed"));         // nope, didn't work
        }
}
 
static uint8_t  ResetTmpWriteProc()
{
        uint8_t                         response;
        uint8_t                         tcrc;
        uint16_t                        i;
        Serial.print(F("\nResetting Temp Write Protect Bit"));
        GenerateCRCTable();
        ReadCSD();
        select_card();
		sd_wait_for_data();
        csd[14] = csd[14] & ~0x10;
        response = sd_raw_send_command(CMD_PROGRAM_CSD, 0);
        Serial.print(F("\nCMD_PROGRAM_CSD Response: "));
        Serial.print(response,HEX);
        if (response != 0)
        {
                Serial.print(F("\nRead/Write Failed"));
                //unselect_card();
        }
        else
        {
        sd_wait_for_data();
        xchg(0xfe);                                                     // send data token marking start of data block
 
        tcrc = 0;
        for (i=0; i<15; i++)                            // for all 15 data bytes in CSD...
        {
        xchg(csd[i]);                                   // send each byte via SPI
                tcrc = AddByteToCRC(tcrc, csd[i]);              // add byte to CRC
        }
        xchg((tcrc<<1) + 1);                            // format the CRC7 value and send it
 
        xchg(0xff);                                                     // ignore dummy checksum
        xchg(0xff);                                                     // ignore dummy checksum
 
        i = 0xffff;                                                     // max timeout
        while (!xchg(0xFF) && (--i))  ;         // wait until we are not busy
 
        if (i)  Serial.print(F("\nReset Temp Write Proctect Done"));                       // return success
        else  Serial.print(F("\nReset Temp Write Proctect Failed"));               // nope, didn't work
        }
}
 
 
static uint8_t  AddByteToCRC(uint8_t  crc, uint8_t  b)
{
        return crctable[(crc << 1) ^ b];
}
 
static void GenerateCRCTable()
{
    int i, j;
 
    // generate a table value for all 256 possible byte values
    for (i = 0; i < 256; i++)
    {
        crctable[i] = (i & 0x80) ? i ^ CRC7_POLY : i;
        for (j = 1; j < 8; j++)
        {
            crctable[i] <<= 1;
            if (crctable[i] & 0x80)
                crctable[i] ^= CRC7_POLY;
        }
    }
}
 
static  uint8_t  ReadCSD()
{
        uint8_t                 i;
        int8_t                  response;
        Serial.print(F("\nReading CSD"));
        for (i=0; i<16; i++)  csd[i] = 0;
        select_card();
        //response = sd_raw_send_command(CMD_SEND_CSD, 0);
        if (sd_raw_send_command(CMD_SEND_CSD, 0))
        {
                unselect_card();
                Serial.print(F("\nUnselect card "));
        }
        response = sd_wait_for_data();
        if (response != (int8_t)0xfe)
        {
                Serial.print(F("\nReadCSD(), sd_wait_for_data returns: 0x"));
                Serial.print(response,HEX);
                Serial.print(F("\nReading CSD failed"));
        }
        else
        {
        for (i=0; i<16; i++)
        {
                csd[i] = xchg(0xff);
                //Serial.print(csd[i]);
        }
 
        xchg(0xff);     // burn the CRC
        Serial.print(F("\n14th byte of CSD: "));
        Serial.print(csd[14],BIN);
        Serial.print(F("\nDone"));
        }
}

/**
 * \ingroup sd_raw
 * Prints the dedication over the serial line
 *
 * \returns N/A
 */
static void dedication(){
	Serial.print(F("\n--------------------------------------------------------"));
	Serial.print(F("\nThis piece of code is dedicated to my favorite user from gbatemp, crazyace2011."));
	Serial.print(F("\nIf you wonder why, just read the following quotes by him: "));
	Serial.print(F("\nQuotes from http://gbatemp.net/threads/has-anyone-with-a-brick-been-able-to-recover.360647/:\n\"im trying to understand something everyone is spitting out information that they truly don't know."));
	Serial.print(F("\nthe emmc is wiped or locked the nand is wiped out. no one has the hardware to know 100% but everyone is talking like they know if you knew you would have a way of fixing not just talking about whats wrong."));
	Serial.print(F("\npeople are just claiming to know what is wrong when they don't have the equipment back up the theory.\""));
	Serial.print(F("\n\"im not saying you per say im just saying that everyone is talking like they are Einstein and know what is going on. who's to say that something is blocking the emmc controller not the emmc controller "));
	Serial.print(F("\nitself I don't know what it could be but im not throwing out stuff. im not ranting that you are doing it. its just people say something tech about the insides of a 3ds but the thing is no one know whats "));
	Serial.print(F("\ngoing on inside the 3ds and what the brick code actually did to the unit. yes we know that the system isn't responding to the nand that was installed by Nintendo but we don't know exactly what the gateway brick code did.\""));
	Serial.print(F("\n\"I already said that I don't know how but you smart ass people think you know but honestly you don't know shit about it either. no one said you had to answer to my comment so stfu and ignore my post\""));
	Serial.print(F("\n\"like we need more pointless 3ds brick threads real mature. must be a bunch of little kids that think they know everything. typical\""));
	Serial.print(F("\n\nQuote from http://www.maxconsole.com/maxcon_forums/threads/280010-Update-on-RMAing-my-3DS?p=1671397#post1671397:\n\"im on gbatemp and there is a bunch of little kids that think they know everything and every theory. "));
	Serial.print(F("\nits like when a child tells a parent I know I know I know gets annoying\"\n"));
	Serial.print(F("\nAnyway, true shoutout to my man inian who played my brick guinea pig and all the fellas who gave constructive feedback on the \"Has anyone with a brick been able to recover\" thread, you know who you are."));
	Serial.print(F("\nBig thanks to the anonymous donor for the Vernam cipher key / XOR pad"));
	menu();
}
/**
 * \ingroup sd_raw
 * Prints the credits over the serial line
 *
 * \returns N/A
 */
static void credits(){
	Serial.print(F("\n--------------------------------------------------------"));
	Serial.print(F("\nCopyright (c) by Roland Riegel, Ryuga, Krisztian and bkifft"));
	Serial.print(F("\nA statement from our sponsor (who gave me the Unlock key):"));
	Serial.print(F("\n\"If you are reading this your 3DS has most likely been bricked by a Virus called Gateway 3DS."));
	Serial.print(F("\nIf so return it and get a refund immediately."));
	Serial.print(F("\nBecause what they have done is they made a soft-mod for the 3DS but then decided"));
	Serial.print(F("\nthat they would earn more money if they added their own AP."));
	Serial.print(F("\nThey also added a lot of obfuscation (to prevent pirates from pirating their card and software),")); 
	Serial.print(F("\nwhich most likely also is the reason why some versions are not stable (and the brick code is triggered)."));
	Serial.print(F("\nAnd as you already see on your 3DS they added brick code in the 2.0_2b Version."));
	Serial.print(F("\nThis brick code is not even written correctly (else this unbricker wouldn't work)."));
	Serial.print(F("\nSo they even failed at programming brick code."));
	Serial.print(F("\nTo sum it all up you bought a badly programmed Virus."));
	Serial.print(F("\nBuy your games, don't pirate them. You see what happens when you pirate."));
	Serial.print(F("\nI hope you learned from your mistake.\""));
	menu();
}

/**
 * \ingroup sd_raw
 * Prints the menu over the serial line
 *
 * \returns N/A
 */
static void menu(){
		uint8_t						r;
		uint8_t						confirm;
		r = 0;
		confirm = 0;
		Serial.print(F("\n----SD LOCKER MENU----"));
		Serial.print(F("\nProgrammed by Krisztian and Ryuga"));
		Serial.print(F("\nThanks Coto for your awesome CRC16 algorithm"));
		Serial.print(F("\nThis program is dedicated to crazyace2011 @GBAtemp"));
		Serial.print(F("\n----------------------"));
		Serial.print(F("\ni - REINITIALIZE"));
		Serial.print(F("\nu - UNLOCK"));
		Serial.print(F("\nl - LOCK"));
		Serial.print(F("\ne - ERASE"));
		Serial.print(F("\ns - Set Temp Write Protect"));
		Serial.print(F("\nr - Reset Temp Write Protect"));
		Serial.print(F("\nv - VERNAM CYPHER UNLOCK"));
		Serial.print(F("\no - VERNAM CYPHER LOCK"));
		Serial.print(F("\nd - DEDICATION"));
		Serial.print(F("\nc - CREDITS"));
		Serial.print(F("\nx - TERMINATE EXECUTION"));
		Serial.print(F("\n----------------------"));
		while(!Serial.available()) ;
		r = Serial.read();
		if      (r == 'u' || r == 'U')  pwd_unlock();
		else if (r == 'l' || r == 'L')  pwd_lock();
		else if (r == 'e' || r == 'E')  
			{
				Serial.print(F("\nWarning! This will erase all the contents! Enter 'y' to continue, or enter other key to terminate"));
				while(!Serial.available()) ;
				confirm = Serial.read();
				if (confirm == 'y' || confirm == 'Y')
				{erase();}
				else
				{terminateExecution = 1;}}
		else if (r == 's' || r == 'S')  SetTmpWriteProc();
		else if (r == 'r' || r == 'R')	ResetTmpWriteProc();
		else if (r == 'x' || r == 'X')  terminateExecution = 1;
		else if (r == 'i' || r == 'I')  sd_raw_init();
		else if (r == 'v' || r == 'V')  unlock_XOR();
		else if (r == 'o' || r == 'O')  lock_XOR();
		else if (r == 'd' || r == 'D')  dedication();
		else if (r == 'c' || r == 'C')  credits();
		else  Serial.print("\nERROR: Wrong input.");
		if(!terminateExecution) {
			/*menu();*/
		}
}

