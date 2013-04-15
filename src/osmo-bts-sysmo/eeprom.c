// $Id: $
/****************************************************************************
 *
 *    ****                              I
 *   ******                            ***
 *   *******                           ****
 *   ********    ****  ****     **** *********    ******* ****    ***********
 *   *********   ****  ****     **** *********  **************  *************
 *   **** *****  ****  ****     ****   ****    *****    ****** *****     ****
 *   ****  ***** ****  ****     ****   ****   *****      ****  ****      ****
 *  ****    *********  ****     ****   ****   ****       ****  ****      ****
 *  ****     ********  ****    ****I  ****    *****     *****  ****      ****
 *  ****      ******   ***** ******   *****    ****** *******  ****** *******
 *  ****        ****   ************    ******   *************   *************
 *  ****         ***     ****  ****     ****      *****  ****     *****  ****
 *                                                                       ****
 *          I N N O V A T I O N  T O D A Y  F O R  T O M M O R O W       ****
 *                                                                        ***
 *
 ************************************************************************//**
 *
 * @file   eeprom.c
 * @brief  SuperFemto EEPROM interface.
 *
 * Author   : Yves Godin
 * Date     : 2012
 * $Revision: $
 *
 * Copyright (c) Nutaq. 2012
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ***************************************************************************
 *
 * "$Revision: $"
 * "$Name: $"
 * "$Date: $"
 *
 ***************************************************************************/
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "eeprom.h"

//#define DISP_ERROR  1

/****************************************************************************
 *                            Private constants                             *
 ****************************************************************************/

/**
 * EEPROM device file
 */
#define EEPROM_DEV  "/sys/bus/i2c/devices/i2c-1/1-0050/eeprom"

/**
 * EEPROM configuration start address
 */
#define EEPROM_CFG_START_ADDR   0x0100

/**
 * EEPROM configuration max size
 */
#define EEPROM_CFG_MAX_SIZE     (0x2000 - EEPROM_CFG_START_ADDR)

/**
 * EEPROM config magic ID
 */
#define EEPROM_CFG_MAGIC_ID     0x53464548

/**
 * EEPROM section ID
 */
typedef enum
{
    EEPROM_SID_SYSINFO          = 0x1000,       ///< System information
    EEPROM_SID_RFCLOCK_CAL      = 0x2000,       ///< RF Clock Calibration
    EEPROM_SID_GSM850_TXCAL     = 0x3000,       ///< GSM-850 TX Calibration Table
    EEPROM_SID_GSM850_RXUCAL    = 0x3010,       ///< GSM-850 RX Uplink Calibration Table
    EEPROM_SID_GSM850_RXDCAL    = 0x3020,       ///< GSM-850 RX Downlink Calibration Table
    EEPROM_SID_GSM900_TXCAL     = 0x3100,       ///< GSM-900 TX Calibration Table
    EEPROM_SID_GSM900_RXUCAL    = 0x3110,       ///< GSM-900 RX Uplink Calibration Table
    EEPROM_SID_GSM900_RXDCAL    = 0x3120,       ///< GSM-900 RX Downlink Calibration Table
    EEPROM_SID_DCS1800_TXCAL    = 0x3200,       ///< DCS-1800 TX Calibration Table
    EEPROM_SID_DCS1800_RXUCAL   = 0x3210,       ///< DCS-1800 RX Uplink Calibration Table
    EEPROM_SID_DCS1800_RXDCAL   = 0x3220,       ///< DCS-1800 RX Downlink Calibration Table
    EEPROM_SID_PCS1900_TXCAL    = 0x3300,       ///< PCS-1900 TX Calibration Table
    EEPROM_SID_PCS1900_RXUCAL   = 0x3310,       ///< PCS-1900 RX Uplink Calibration Table
    EEPROM_SID_PCS1900_RXDCAL   = 0x3320        ///< PCS-1900 RX Downlink Calibration Table
} eeprom_SID_t;

/****************************************************************************
 *                              Private types                               *
 ****************************************************************************/

/**
 * TX calibration table (common part)
 */
typedef struct
{
    uint16_t u16SectionID;                      ///< Section ID
    uint16_t u16Crc;                            ///< Parity
    uint32_t u32Time;                           ///< Epoch time

    int16_t  sfixTxGainGmsk[80];                ///< [Q10.5] Gain setting for GMSK output level from +50dBm to -29 dBm
    int16_t  sfixTx8PskCorr;                    ///< [Q6.9] Gain adjustment for 8 PSK (default to +3.25 dB)
    int16_t  sfixTxExtAttCorr[31];              ///< [Q6.9] Gain adjustment for external attenuator (0:@1dB, 1:@2dB, ..., 31:@32dB)
    int16_t  sfixTxRollOffCorr[0];              ///< [Q6.9] Gain correction for each ARFCN
} __attribute__((packed)) eeprom_CfgTxCal_t;

/**
 * RX calibration table (common part)
 */
typedef struct
{
    uint16_t u16SectionID;                      ///< Section ID
    uint16_t u16Crc;                            ///< Parity
    uint32_t u32Time;                           ///< Epoch time

    uint16_t u16IqImbalMode;                    ///< IQ imbalance mode (0:off, 1:on, 2:auto)
    uint16_t u16IqImbalCorr[4];                 ///< IQ imbalance compensation

    int16_t sfixExtRxGain;                      ///< [Q6.9] External RX gain
    int16_t sfixRxMixGainCorr;                  ///< [Q6.9] Mixer gain error compensation
    int16_t sfixRxLnaGainCorr[3];               ///< [Q6.9] LNA gain error compensation (1:@-12 dB, 2:@-24 dB, 3:@-36 dB)
    int16_t sfixRxRollOffCorr[0];               ///< [Q6.9] Frequency roll-off compensation
} __attribute__((packed)) eeprom_CfgRxCal_t;

/**
 * EEPROM configuration area format
 */
typedef struct
{
    struct
    {
        uint32_t u32MagicId;                    ///< Magic ID (0x53464548)
        uint32_t u16Version : 16;               ///< Header format version (v1)
        uint32_t            : 16;               ///< unused
    } hdr;

    union
    {
        /** EEPROM Format V1 */
        struct
        {
            /** System information */
            struct
            {
                uint16_t u16SectionID;          ///< Section ID
                uint16_t u16Crc;                ///< Parity
                uint32_t u32Time;               ///< Epoch time

                char szSn[16];                  ///< Serial number
                uint32_t u8Rev      :  8;       ///< Board revision
                uint32_t u2Tcxo     :  2;       ///< TCXO present       (0:absent, 1:present, x:unknows)
                uint32_t u2Ocxo     :  2;       ///< OCXO present       (0:absent, 1:present, x:unknows)
                uint32_t u2GSM850   :  2;       ///< GSM-850 supported  (0:unsupported, 1:supported, x:unknows)
                uint32_t u2GSM900   :  2;       ///< GSM-900 supported  (0:unsupported, 1:supported, x:unknows)
                uint32_t u2DCS1800  :  2;       ///< GSM-1800 supported (0:unsupported, 1:supported, x:unknows)
                uint32_t u2PCS1900  :  2;       ///< GSM-1900 supported (0:unsupported, 1:supported, x:unknows)
                uint32_t            : 12;       ///< unused
            } __attribute__((packed)) sysInfo;

            /** RF Clock configuration */
            struct
            {
                uint16_t u16SectionID;          ///< Section ID
                uint16_t u16Crc;                ///< Parity
                uint32_t u32Time;               ///< Epoch time

                int      i24ClkCor  :24;        ///< Clock correction value in PPB.
                uint32_t u8ClkSrc   : 8;        ///< Clock source (0:None, 1:OCXO, 2:TCXO, 3:External, 4:GPS PPS, 5:reserved, 6:RX, 7:Edge)
            } __attribute__((packed)) rfClk;

            /** GSM-850 TX Calibration Table */
            eeprom_CfgTxCal_t gsm850TxCal;
            uint16_t __gsm850TxCalMem[124];

            /** GSM-850 RX Uplink Calibration Table */
            eeprom_CfgRxCal_t gsm850RxuCal;
            uint16_t __gsm850RxuCalMem[124];

            /** GSM-850 RX Downlink Calibration Table */
            eeprom_CfgRxCal_t gsm850RxdCal;
            uint16_t __gsm850RxdCalMem[124];

            /** GSM-900 TX Calibration Table */
            eeprom_CfgTxCal_t gsm900TxCal;
            uint16_t __gsm900TxCalMem[194];

            /** GSM-900 RX Uplink Calibration Table */
            eeprom_CfgRxCal_t gsm900RxuCal;
            uint16_t __gsm900RxuCalMem[194];

            /** GSM-900 RX Downlink Calibration Table */
            eeprom_CfgRxCal_t gsm900RxdCal;
            uint16_t __gsm900RxdCalMem[194];

            /** DCS-1800 TX Calibration Table */
            eeprom_CfgTxCal_t dcs1800TxCal;
            uint16_t __dcs1800TxCalMem[374];

            /** DCS-1800 RX Uplink Calibration Table */
            eeprom_CfgRxCal_t dcs1800RxuCal;
            uint16_t __dcs1800RxuCalMem[374];

            /** DCS-1800 RX Downlink Calibration Table */
            eeprom_CfgRxCal_t dcs1800RxdCal;
            uint16_t __dcs1800RxdCalMem[374];

            /** PCS-1900 TX Calibration Table */
            eeprom_CfgTxCal_t pcs1900TxCal;
            uint16_t __pcs1900TxCalMem[299];

            /** PCS-1900 RX Uplink Calibration Table */
            eeprom_CfgRxCal_t pcs1900RxuCal;
            uint16_t __pcs1900RxuCalMem[299];

            /** PCS-1900 RX Downlink Calibration Table */
            eeprom_CfgRxCal_t pcs1900RxdCal;
            uint16_t __pcs1900RxdCalMem[299];

        } __attribute__((packed)) v1;
    } __attribute__((packed)) cfg;
} __attribute__((packed)) eeprom_Cfg_t;



/****************************************************************************
 *                        Private routine prototypes                        *
 ****************************************************************************/

static int eeprom_read( int addr, int size, char *pBuff );
static int eeprom_write( int addr, int size, const char *pBuff );
static uint16_t eeprom_crc( uint8_t *pu8Data, int len );


/****************************************************************************
 *                             Public functions                             *
 ****************************************************************************/

/****************************************************************************
 * Function : eeprom_ResetCfg
 ************************************************************************//**
 *
 * This function reset the content of the EEPROM config area.
 *
 * @return
 *    0 if or an error core.
 *
 ****************************************************************************/
eeprom_Error_t eeprom_ResetCfg( void )
{
    int err;
    eeprom_Cfg_t ee;

    // Clear the structure
    memset( &ee, 0xFF, sizeof(eeprom_Cfg_t) );

    // Init the header
    ee.hdr.u32MagicId = EEPROM_CFG_MAGIC_ID;
    ee.hdr.u16Version = 1;

    // Write it to the EEPROM
    err = eeprom_write( EEPROM_CFG_START_ADDR, sizeof(ee.hdr) + sizeof(ee.cfg.v1), (const char *) &ee );
    if ( err != sizeof(ee.hdr) + sizeof(ee.cfg.v1) )
    {
        return EEPROM_ERR_DEVICE;
    }
    return EEPROM_SUCCESS;
}


/****************************************************************************
 * Function : eeprom_ReadSysInfo
 ************************************************************************//**
 *
 * This function reads the system information from the EEPROM.
 *
 * @param [inout] pTime
 *    Pointer to a system info structure.
 *
 * @param [inout] pSysInfo
 *    Pointer to a system info structure.
 *
 * @return
 *    0 if or an error core.
 *
 ****************************************************************************/
eeprom_Error_t eeprom_ReadSysInfo( eeprom_SysInfo_t *pSysInfo )
{
    int err;
    eeprom_Cfg_t ee;

    // Get a copy of the EEPROM header
    err = eeprom_read( EEPROM_CFG_START_ADDR, sizeof(ee.hdr), (char *) &ee.hdr );
    if ( err != sizeof(ee.hdr) )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
        return EEPROM_ERR_DEVICE;
    }

    // Validate the header magic ID
    if ( ee.hdr.u32MagicId != EEPROM_CFG_MAGIC_ID )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Invalid EEPROM format\n" );
#endif
        return EEPROM_ERR_INVALID;
    }

    switch ( ee.hdr.u16Version )
    {
        case 1:
        {
            // Get a copy of the EEPROM section
            err = eeprom_read( EEPROM_CFG_START_ADDR + ((uint32_t)&ee.cfg.v1.sysInfo - (uint32_t)&ee), sizeof(ee.cfg.v1.sysInfo), (char *)&ee.cfg.v1.sysInfo );
            if ( err != sizeof(ee.cfg.v1.sysInfo) )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
                return EEPROM_ERR_DEVICE;
            }
            
            // Validate the ID
            if ( ee.cfg.v1.sysInfo.u16SectionID != EEPROM_SID_SYSINFO )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Uninitialized data section\n" );
#endif
                return EEPROM_ERR_UNAVAILABLE;
            }

            // Validate the CRC
            if ( eeprom_crc( (uint8_t *)&ee.cfg.v1.sysInfo.u32Time, sizeof(ee.cfg.v1.sysInfo) - 2 * sizeof(uint16_t) ) != ee.cfg.v1.sysInfo.u16Crc )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Parity error\n" );
#endif
                return EEPROM_ERR_PARITY;
            }

            // Expand the content of the section
            memcpy( (void *)pSysInfo->szSn, ee.cfg.v1.sysInfo.szSn, sizeof(pSysInfo->szSn) );
            pSysInfo->u8Rev     = ee.cfg.v1.sysInfo.u8Rev;
            pSysInfo->u8Tcxo    = ee.cfg.v1.sysInfo.u2Tcxo;
            pSysInfo->u8Ocxo    = ee.cfg.v1.sysInfo.u2Ocxo;
            pSysInfo->u8GSM850  = ee.cfg.v1.sysInfo.u2GSM850;
            pSysInfo->u8GSM900  = ee.cfg.v1.sysInfo.u2GSM900;
            pSysInfo->u8DCS1800 = ee.cfg.v1.sysInfo.u2DCS1800;
            pSysInfo->u8PCS1900 = ee.cfg.v1.sysInfo.u2PCS1900;
            break;
        }

        default:
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Unsupported header version\n" );
#endif
            return EEPROM_ERR_UNSUPPORTED;
        }
    }
    return EEPROM_SUCCESS;
}


/****************************************************************************
 * Function : eeprom_WriteSysInfo
 ************************************************************************//**
 *
 * This function writes the system information to the EEPROM.
 *
 * @param [in] pSysInfo
 *    Pointer to the system info structure to be written.
 *
 * @return
 *    0 if or an error core.
 *
 ****************************************************************************/
eeprom_Error_t eeprom_WriteSysInfo( const eeprom_SysInfo_t *pSysInfo )
{
    int err;
    eeprom_Cfg_t ee;

    // Get a copy of the EEPROM header
    err = eeprom_read( EEPROM_CFG_START_ADDR, sizeof(ee.hdr), (char *) &ee.hdr );
    if ( err != sizeof(ee.hdr) )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
        return EEPROM_ERR_DEVICE;
    }

    // Validate the header magic ID
    if ( ee.hdr.u32MagicId != EEPROM_CFG_MAGIC_ID )
    {
        // Init the header
        ee.hdr.u32MagicId = EEPROM_CFG_MAGIC_ID;
        ee.hdr.u16Version = 1;

        // Write it to the EEPROM
        err = eeprom_write( EEPROM_CFG_START_ADDR, sizeof(ee.hdr) + sizeof(ee.cfg.v1), (const char *) &ee );
        if ( err != sizeof(ee.hdr) + sizeof(ee.cfg.v1) )
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Error while writing to the EEPROM (%d)\n", err );
#endif
            return EEPROM_ERR_DEVICE;
        }
    }

    switch ( ee.hdr.u16Version )
    {
        case 1:
        {
            ee.cfg.v1.sysInfo.u16SectionID = EEPROM_SID_SYSINFO;
            ee.cfg.v1.sysInfo.u16Crc       = 0;
            ee.cfg.v1.sysInfo.u32Time      = time(NULL);

            // Compress the info
            memcpy( ee.cfg.v1.sysInfo.szSn, pSysInfo->szSn, sizeof(ee.cfg.v1.sysInfo.szSn) );
            ee.cfg.v1.sysInfo.u8Rev     = pSysInfo->u8Rev;
            ee.cfg.v1.sysInfo.u2Tcxo    = pSysInfo->u8Tcxo;
            ee.cfg.v1.sysInfo.u2Ocxo    = pSysInfo->u8Ocxo;
            ee.cfg.v1.sysInfo.u2GSM850  = pSysInfo->u8GSM850;
            ee.cfg.v1.sysInfo.u2GSM900  = pSysInfo->u8GSM900;
            ee.cfg.v1.sysInfo.u2DCS1800 = pSysInfo->u8DCS1800;
            ee.cfg.v1.sysInfo.u2PCS1900 = pSysInfo->u8PCS1900;

            // Add the CRC
            ee.cfg.v1.sysInfo.u16Crc = eeprom_crc( (uint8_t *)&ee.cfg.v1.sysInfo.u32Time, sizeof(ee.cfg.v1.sysInfo) - 2 * sizeof(uint16_t) );

            // Write it to the EEPROM
            err = eeprom_write( EEPROM_CFG_START_ADDR + ((uint32_t)&ee.cfg.v1.sysInfo - (uint32_t)&ee), sizeof(ee.cfg.v1.sysInfo), (const char *) &ee.cfg.v1.sysInfo );
            if ( err != sizeof(ee.cfg.v1.sysInfo) )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Error while writing to the EEPROM (%d)\n", err );
#endif
                return EEPROM_ERR_DEVICE;
            }
            break;
        }

        default:
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Unsupported header version\n" );
#endif
            return EEPROM_ERR_UNSUPPORTED;
        }
    }
    return EEPROM_SUCCESS;
}


/****************************************************************************
 * Function : eeprom_ReadRfClockCal
 ************************************************************************//**
 *
 * This function reads the RF clock calibration data from the EEPROM.
 *
 * @param [inout] pRfClockCal
 *    Pointer to a RF clock calibration structure.
 *
 * @return
 *    0 if or an error core.
 *
 ****************************************************************************/
eeprom_Error_t eeprom_ReadRfClockCal( eeprom_RfClockCal_t *pRfClockCal )
{
    int err;
    eeprom_Cfg_t ee;

    // Get a copy of the EEPROM header
    err = eeprom_read( EEPROM_CFG_START_ADDR, sizeof(ee), (char *) &ee );
    if ( err != sizeof(ee) )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
        return EEPROM_ERR_DEVICE;
    }

    // Validate the header magic ID
    if ( ee.hdr.u32MagicId != EEPROM_CFG_MAGIC_ID )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Invalid EEPROM format\n" );
#endif
        return EEPROM_ERR_INVALID;
    }

    switch ( ee.hdr.u16Version )
    {
        case 1:
        {
            // Get a copy of the EEPROM section 
            err = eeprom_read( EEPROM_CFG_START_ADDR + ((uint32_t)&ee.cfg.v1.rfClk - (uint32_t)&ee), sizeof(ee.cfg.v1.rfClk), (char *)&ee.cfg.v1.rfClk );
            if ( err != sizeof(ee.cfg.v1.rfClk) )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
                return EEPROM_ERR_DEVICE;
            }

            // Validate the ID
            if ( ee.cfg.v1.rfClk.u16SectionID != EEPROM_SID_RFCLOCK_CAL )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Uninitialized data section\n" );
#endif
                return EEPROM_ERR_UNAVAILABLE;
            }

            // Validate the CRC
            if ( eeprom_crc( (uint8_t *)&ee.cfg.v1.rfClk.u32Time, sizeof(ee.cfg.v1.rfClk) - 2 * sizeof(uint16_t) ) != ee.cfg.v1.rfClk.u16Crc )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Parity error\n" );
#endif
                return EEPROM_ERR_PARITY;
            }

            // Expand the content of the section
            pRfClockCal->iClkCor  = ee.cfg.v1.rfClk.i24ClkCor;
            pRfClockCal->u8ClkSrc = ee.cfg.v1.rfClk.u8ClkSrc;
            break;
        }

        default:
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Unsupported header version\n" );
#endif
            return EEPROM_ERR_UNSUPPORTED;
        }
    }
    return EEPROM_SUCCESS;
}


/****************************************************************************
 * Function : eeprom_WriteRfClockCal
 ************************************************************************//**
 *
 * This function writes the RF clock calibration data to the EEPROM.
 *
 * @param [in] pSysInfo
 *    Pointer to the system info structure to be written.
 *
 * @return
 *    0 if or an error core.
 *
 ****************************************************************************/
eeprom_Error_t eeprom_WriteRfClockCal( const eeprom_RfClockCal_t *pRfClockCal )
{
    int err;
    eeprom_Cfg_t ee;

    // Get a copy of the EEPROM header
    err = eeprom_read( EEPROM_CFG_START_ADDR, sizeof(ee.hdr), (char *) &ee.hdr );
    if ( err != sizeof(ee.hdr) )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
        return EEPROM_ERR_DEVICE;
    }

    // Validate the header magic ID
    if ( ee.hdr.u32MagicId != EEPROM_CFG_MAGIC_ID )
    {
        // Init the header
        ee.hdr.u32MagicId = EEPROM_CFG_MAGIC_ID;
        ee.hdr.u16Version = 1;

        // Write it to the EEPROM
        err = eeprom_write( EEPROM_CFG_START_ADDR, sizeof(ee.hdr) + sizeof(ee.cfg.v1), (const char *) &ee );
        if ( err != sizeof(ee.hdr) + sizeof(ee.cfg.v1) )
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Error while writing to the EEPROM (%d)\n", err );
#endif
            return EEPROM_ERR_DEVICE;
        }
    }

    switch ( ee.hdr.u16Version )
    {
        case 1:
        {
            ee.cfg.v1.rfClk.u16SectionID = EEPROM_SID_RFCLOCK_CAL;
            ee.cfg.v1.rfClk.u16Crc       = 0;
            ee.cfg.v1.rfClk.u32Time      = time(NULL);

            // Compress the info
            ee.cfg.v1.rfClk.i24ClkCor = pRfClockCal->iClkCor;
            ee.cfg.v1.rfClk.u8ClkSrc  = pRfClockCal->u8ClkSrc;

            // Add the CRC
            ee.cfg.v1.rfClk.u16Crc = eeprom_crc( (uint8_t *)&ee.cfg.v1.rfClk.u32Time, sizeof(ee.cfg.v1.rfClk) - 2 * sizeof(uint16_t) );

            // Write it to the EEPROM
            err = eeprom_write( EEPROM_CFG_START_ADDR + ((uint32_t)&ee.cfg.v1.rfClk - (uint32_t)&ee), sizeof(ee.cfg.v1.rfClk), (const char *) &ee.cfg.v1.rfClk );
            if ( err != sizeof(ee.cfg.v1.rfClk) )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Error while writing to the EEPROM (%d)\n", err );
#endif
                return EEPROM_ERR_DEVICE;
            }
            break;
        }

        default:
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Unsupported header version\n" );
#endif
           return EEPROM_ERR_UNSUPPORTED;
        }
    }
    return EEPROM_SUCCESS;
}


/****************************************************************************
 * Function : eeprom_ReadTxCal
 ************************************************************************//**
 *
 * This function reads the TX calibration tables for the specified band from
 * the EEPROM.
 *
 * @param [in] iBand
 *    GSM band (0:GSM-850, 1:GSM-900, 2:DCS-1800, 3:PCS-1900).
 *
 * @param [inout] pTxCal
 *    Pointer to a TX calibration table structure.
 *
 * @return
 *    0 if or an error core.
 *
 ****************************************************************************/
eeprom_Error_t eeprom_ReadTxCal( int iBand, eeprom_TxCal_t *pTxCal )
{
    int i;
    int err;
    int size;
    int nArfcn;
    eeprom_Cfg_t ee;
    eeprom_SID_t sId;
    eeprom_CfgTxCal_t *pCfgTxCal = NULL;

    // Get a copy of the EEPROM header
    err = eeprom_read( EEPROM_CFG_START_ADDR, sizeof(ee.hdr), (char *)&ee.hdr );
    if ( err != sizeof(ee.hdr) )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
        return EEPROM_ERR_DEVICE;
    }

    // Validate the header magic ID
    if ( ee.hdr.u32MagicId != EEPROM_CFG_MAGIC_ID )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Invalid EEPROM format\n" );
#endif
        return EEPROM_ERR_INVALID;
    }

    switch ( ee.hdr.u16Version )
    {
        case 1:
        {
            switch ( iBand )
            {
                case 0:
                    nArfcn = 124;
                    sId = EEPROM_SID_GSM850_TXCAL;
                    pCfgTxCal = &ee.cfg.v1.gsm850TxCal;
                    size = sizeof(ee.cfg.v1.gsm850TxCal) + sizeof(ee.cfg.v1.__gsm850TxCalMem);
                    break;
                case 1:
                    nArfcn = 194;
                    sId = EEPROM_SID_GSM900_TXCAL;
                    pCfgTxCal = &ee.cfg.v1.gsm900TxCal;
                    size = sizeof(ee.cfg.v1.gsm900TxCal) + sizeof(ee.cfg.v1.__gsm900TxCalMem);
                    break;
                case 2:
                    nArfcn = 374;
                    sId = EEPROM_SID_DCS1800_TXCAL;
                    pCfgTxCal = &ee.cfg.v1.dcs1800TxCal;
                    size = sizeof(ee.cfg.v1.dcs1800TxCal) + sizeof(ee.cfg.v1.__dcs1800TxCalMem);
                    break;
                case 3:
                    nArfcn = 299;
                    sId = EEPROM_SID_PCS1900_TXCAL;
                    pCfgTxCal = &ee.cfg.v1.pcs1900TxCal;
                    size = sizeof(ee.cfg.v1.pcs1900TxCal) + sizeof(ee.cfg.v1.__pcs1900TxCalMem);
                    break;
                default:
#ifdef DISP_ERROR
                    fprintf( stderr, "Invalid GSM band specified (%d)\n", iBand );
#endif
                    return EEPROM_ERR_INVALID;
            }

            // Get a copy of the EEPROM section
            err = eeprom_read( EEPROM_CFG_START_ADDR + ((uint32_t)pCfgTxCal - (uint32_t)&ee), size, (char *)pCfgTxCal );
            if ( err != size )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
                return EEPROM_ERR_DEVICE;
            }

            // Validate the ID
            if ( pCfgTxCal->u16SectionID != sId )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Uninitialized data section\n" );
#endif
                return EEPROM_ERR_UNAVAILABLE;
            }

            // Validate the CRC
            if ( eeprom_crc( (uint8_t *)&pCfgTxCal->u32Time, size - 2 * sizeof(uint16_t) ) != pCfgTxCal->u16Crc )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Parity error\n" );
#endif
                return EEPROM_ERR_PARITY;
            }

            // Expand the content of the section
            for ( i = 0; i < 80; i++ )
            {
                pTxCal->fTxGainGmsk[i] = (float)pCfgTxCal->sfixTxGainGmsk[i] * 0.03125f;
            }
            pTxCal->fTx8PskCorr = (float)pCfgTxCal->sfixTx8PskCorr * 0.001953125f;
            for ( i = 0; i < 31; i++ )
            {
                pTxCal->fTxExtAttCorr[i] = (float)pCfgTxCal->sfixTxExtAttCorr[i] * 0.001953125f;
            }
            for ( i = 0; i < nArfcn; i++ )
            {
                pTxCal->fTxRollOffCorr[i] = (float)pCfgTxCal->sfixTxRollOffCorr[i] * 0.001953125f;
            }
            break;
        }

        default:
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Unsupported header version\n" );
#endif
            return EEPROM_ERR_UNSUPPORTED;
        }
    }
    return EEPROM_SUCCESS;
}


/****************************************************************************
 * Function : eeprom_WriteTxCal
 ************************************************************************//**
 *
 * This function writes the TX calibration tables for the specified band to
 * the EEPROM.
 *
 * @param [in] iBand
 *    GSM band (0:GSM-850, 1:GSM-900, 2:DCS-1800, 3:PCS-1900).
 *
 * @param [in] pTxCal
 *    Pointer to a TX calibration table structure.
 *
 * @return
 *    0 if or an error core.
 *
 ****************************************************************************/
eeprom_Error_t eeprom_WriteTxCal( int iBand, const eeprom_TxCal_t *pTxCal )
{
    int i;
    int err;
    int size;
    int nArfcn;
    eeprom_Cfg_t ee;
    eeprom_SID_t sId;
    eeprom_CfgTxCal_t *pCfgTxCal = NULL;

    // Get a copy of the EEPROM header
    err = eeprom_read( EEPROM_CFG_START_ADDR, sizeof(ee.hdr), (char *) &ee.hdr );
    if ( err != sizeof(ee.hdr) )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
        return EEPROM_ERR_DEVICE;
    }

    // Validate the header magic ID
    if ( ee.hdr.u32MagicId != EEPROM_CFG_MAGIC_ID )
    {
        // Init the header
        ee.hdr.u32MagicId = EEPROM_CFG_MAGIC_ID;
        ee.hdr.u16Version = 1;

        // Write it to the EEPROM
        err = eeprom_write( EEPROM_CFG_START_ADDR, sizeof(ee.hdr) + sizeof(ee.cfg.v1), (const char *) &ee );
        if ( err != sizeof(ee.hdr) + sizeof(ee.cfg.v1) )
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Error while writing to the EEPROM (%d)\n", err );
#endif
            return EEPROM_ERR_DEVICE;
        }
    }

    switch ( ee.hdr.u16Version )
    {
        case 1:
        {
            int32_t fixVal;

            switch ( iBand )
            {
                case 0:
                    nArfcn = 124;
                    sId = EEPROM_SID_GSM850_TXCAL;
                    pCfgTxCal = &ee.cfg.v1.gsm850TxCal;
                    size = sizeof(ee.cfg.v1.gsm850TxCal) + sizeof(ee.cfg.v1.__gsm850TxCalMem);
                    break;
                case 1:
                    nArfcn = 194;
                    sId = EEPROM_SID_GSM900_TXCAL;
                    pCfgTxCal = &ee.cfg.v1.gsm900TxCal;
                    size = sizeof(ee.cfg.v1.gsm900TxCal) + sizeof(ee.cfg.v1.__gsm900TxCalMem);
                    break;
                case 2:
                    nArfcn = 374;
                    sId = EEPROM_SID_DCS1800_TXCAL;
                    pCfgTxCal = &ee.cfg.v1.dcs1800TxCal;
                    size = sizeof(ee.cfg.v1.dcs1800TxCal) + sizeof(ee.cfg.v1.__dcs1800TxCalMem);
                    break;
                case 3:
                    nArfcn = 299;
                    sId = EEPROM_SID_PCS1900_TXCAL;
                    pCfgTxCal = &ee.cfg.v1.pcs1900TxCal;
                    size = sizeof(ee.cfg.v1.pcs1900TxCal) + sizeof(ee.cfg.v1.__pcs1900TxCalMem);
                    break;
                default:
#ifdef DISP_ERROR
                    fprintf( stderr, "Invalid GSM band specified (%d)\n", iBand );
#endif
                    return EEPROM_ERR_INVALID;
            }

            pCfgTxCal->u16SectionID = sId;
            pCfgTxCal->u16Crc       = 0;
            pCfgTxCal->u32Time      = time(NULL);

            // Compress the calibration tables
            for ( i = 0; i < 80; i++ )
            {
                fixVal = (int32_t)(pTxCal->fTxGainGmsk[i] * 32.f + (pTxCal->fTxGainGmsk[i]>0 ? 0.5f:-0.5f));
                     if ( fixVal >  32767 ) pCfgTxCal->sfixTxGainGmsk[i] =  32767;
                else if ( fixVal < -32768 ) pCfgTxCal->sfixTxGainGmsk[i] = -32768;
                else                        pCfgTxCal->sfixTxGainGmsk[i] = (int16_t)fixVal;
            }
            fixVal = (int32_t)(pTxCal->fTx8PskCorr * 512.f + (pTxCal->fTx8PskCorr>0 ? 0.5f:-0.5f));
                 if ( fixVal >  32767 ) pCfgTxCal->sfixTx8PskCorr =  32767;
            else if ( fixVal < -32768 ) pCfgTxCal->sfixTx8PskCorr = -32768;
            else                        pCfgTxCal->sfixTx8PskCorr = (int16_t)fixVal;
            for ( i = 0; i < 31; i++ )
            {
                fixVal = (int32_t)(pTxCal->fTxExtAttCorr[i] * 512.f + (pTxCal->fTxExtAttCorr[i]>0 ? 0.5f:-0.5f));
                     if ( fixVal >  32767 ) pCfgTxCal->sfixTxExtAttCorr[i] =  32767;
                else if ( fixVal < -32768 ) pCfgTxCal->sfixTxExtAttCorr[i] = -32768;
                else                        pCfgTxCal->sfixTxExtAttCorr[i] = (int16_t)fixVal;
            }
            for ( i = 0; i < nArfcn; i++ )
            {
                fixVal = (int32_t)(pTxCal->fTxRollOffCorr[i] * 512.f + (pTxCal->fTxRollOffCorr[i]>0 ? 0.5f:-0.5f));
                     if ( fixVal >  32767 ) pCfgTxCal->sfixTxRollOffCorr[i] =  32767;
                else if ( fixVal < -32768 ) pCfgTxCal->sfixTxRollOffCorr[i] = -32768;
                else                        pCfgTxCal->sfixTxRollOffCorr[i] = (int16_t)fixVal;
            }

            // Add the CRC
            pCfgTxCal->u16Crc = eeprom_crc( (uint8_t *)&pCfgTxCal->u32Time, size - 2 * sizeof(uint16_t) );

            // Write it to the EEPROM
            err = eeprom_write( EEPROM_CFG_START_ADDR + ((uint32_t)pCfgTxCal - (uint32_t)&ee), size, (const char *)pCfgTxCal );
            if ( err != size )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Error while writing to the EEPROM (%d)\n", err );
#endif
                return EEPROM_ERR_DEVICE;
            }
            break;
        }

        default:
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Unsupported header version\n" );
#endif
            return EEPROM_ERR_UNSUPPORTED;
        }
    }
    return EEPROM_SUCCESS;
}


/****************************************************************************
 * Function : eeprom_ReadRxCal
 ************************************************************************//**
 *
 * This function reads the RX calibration tables for the specified band from
 * the EEPROM.
 *
 * @param [in] iBand
 *    GSM band (0:GSM-850, 1:GSM-900, 2:DCS-1800, 3:PCS-1900).
 *
 * @param [in] iUplink
 *    Uplink flag (0:downlink, X:downlink).
 *
 * @param [inout] pRxCal
 *    Pointer to a RX calibration table structure.
 *
 * @return
 *    0 if or an error core.
 *
 ****************************************************************************/
eeprom_Error_t eeprom_ReadRxCal( int iBand, int iUplink, eeprom_RxCal_t *pRxCal )
{
    int i;
    int err;
    int size;
    int nArfcn;
    eeprom_Cfg_t ee;
    eeprom_SID_t sId;
    eeprom_CfgRxCal_t *pCfgRxCal = NULL;

    // Get a copy of the EEPROM header
    err = eeprom_read( EEPROM_CFG_START_ADDR, sizeof(ee.hdr), (char *)&ee.hdr );
    if ( err != sizeof(ee.hdr) )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
        return EEPROM_ERR_DEVICE;
    }

    // Validate the header magic ID
    if ( ee.hdr.u32MagicId != EEPROM_CFG_MAGIC_ID )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Invalid EEPROM format\n" );
#endif
       return EEPROM_ERR_INVALID;
    }

    switch ( ee.hdr.u16Version )
    {
        case 1:
        {
            switch ( iBand )
            {
                case 0:
                    nArfcn = 124;
                    if ( iUplink )
                    {
                        sId = EEPROM_SID_GSM850_RXUCAL;
                        pCfgRxCal = &ee.cfg.v1.gsm850RxuCal;
                        size = sizeof(ee.cfg.v1.gsm850RxuCal) + sizeof(ee.cfg.v1.__gsm850RxuCalMem);
                    }
                    else
                    {
                        sId = EEPROM_SID_GSM850_RXDCAL;
                        pCfgRxCal = &ee.cfg.v1.gsm850RxdCal;
                        size = sizeof(ee.cfg.v1.gsm850RxdCal) + sizeof(ee.cfg.v1.__gsm850RxdCalMem);
                    }
                    break;
                case 1:
                    nArfcn = 194;
                    if ( iUplink )
                    {
                        sId = EEPROM_SID_GSM900_RXUCAL;
                        pCfgRxCal = &ee.cfg.v1.gsm900RxuCal;
                        size = sizeof(ee.cfg.v1.gsm900RxuCal) + sizeof(ee.cfg.v1.__gsm900RxuCalMem);
                    }
                    else
                    {
                        sId = EEPROM_SID_GSM900_RXDCAL;
                        pCfgRxCal = &ee.cfg.v1.gsm900RxdCal;
                        size = sizeof(ee.cfg.v1.gsm900RxdCal) + sizeof(ee.cfg.v1.__gsm900RxdCalMem);
                    }
                    break;
                case 2:
                    nArfcn = 374;
                    if ( iUplink )
                    {
                        sId = EEPROM_SID_DCS1800_RXUCAL;
                        pCfgRxCal = &ee.cfg.v1.dcs1800RxuCal;
                        size = sizeof(ee.cfg.v1.dcs1800RxuCal) + sizeof(ee.cfg.v1.__dcs1800RxuCalMem);
                    }
                    else
                    {
                        sId = EEPROM_SID_DCS1800_RXDCAL;
                        pCfgRxCal = &ee.cfg.v1.dcs1800RxdCal;
                        size = sizeof(ee.cfg.v1.dcs1800RxdCal) + sizeof(ee.cfg.v1.__dcs1800RxdCalMem);
                    }
                    break;
                case 3:
                    nArfcn = 299;
                    if ( iUplink )
                    {
                        sId = EEPROM_SID_PCS1900_RXUCAL;
                        pCfgRxCal = &ee.cfg.v1.pcs1900RxuCal;
                        size = sizeof(ee.cfg.v1.pcs1900RxuCal) + sizeof(ee.cfg.v1.__pcs1900RxuCalMem);
                    }
                    else
                    {
                        sId = EEPROM_SID_PCS1900_RXDCAL;
                        pCfgRxCal = &ee.cfg.v1.pcs1900RxdCal;
                        size = sizeof(ee.cfg.v1.pcs1900RxdCal) + sizeof(ee.cfg.v1.__pcs1900RxdCalMem);
                    }
                    break;
                default:
#ifdef DISP_ERROR
                    fprintf( stderr, "Invalid GSM band specified (%d)\n", iBand );
#endif
                    return EEPROM_ERR_INVALID;
            }

            // Get a copy of the EEPROM section 
            err = eeprom_read( EEPROM_CFG_START_ADDR + ((uint32_t)pCfgRxCal - (uint32_t)&ee), size, (char *)pCfgRxCal );
            if ( err != size )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
                return EEPROM_ERR_DEVICE;
            }
 
            // Validate the ID
            if ( pCfgRxCal->u16SectionID != sId )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Uninitialized data section\n" );
#endif
                return EEPROM_ERR_UNAVAILABLE;
            }

            // Validate the CRC
            if ( eeprom_crc( (uint8_t *)&pCfgRxCal->u32Time, size - 2 * sizeof(uint16_t) ) != pCfgRxCal->u16Crc )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Parity error\n" );
#endif
               return EEPROM_ERR_PARITY;
            }

            // Expand the IQ imbalance mode (0:off, 1:on, 2:auto)
            pRxCal->u8IqImbalMode = pCfgRxCal->u16IqImbalMode;

            // Expand the IQ imbalance compensation
            for ( i = 0; i < 4; i++ )
            {
                pRxCal->u16IqImbalCorr[i] = pCfgRxCal->u16IqImbalCorr[i];
            }

            // Expand the External RX gain
            pRxCal->fExtRxGain = (float)pCfgRxCal->sfixExtRxGain * 0.001953125;

            // Expand the Mixer gain error compensation
            pRxCal->fRxMixGainCorr = (float)pCfgRxCal->sfixRxMixGainCorr * 0.001953125;

            // Expand the LNA gain error compensation (1:@-12 dB, 2:@-24 dB, 3:@-36 dB)
            for ( i = 0; i < 3; i++ )
            {
                pRxCal->fRxLnaGainCorr[i] = (float)pCfgRxCal->sfixRxLnaGainCorr[i] * 0.001953125;
            }

            // Expand the Frequency roll-off compensation
            for ( i = 0; i < nArfcn; i++ )
            {
                pRxCal->fRxRollOffCorr[i] = (float)pCfgRxCal->sfixRxRollOffCorr[i] * 0.001953125;
            }
            break;
        }

        default:
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Unsupported header version\n" );
#endif
            return EEPROM_ERR_UNSUPPORTED;
        }
    }
    return EEPROM_SUCCESS;
}


/****************************************************************************
 * Function : eeprom_WriteRxCal
 ************************************************************************//**
 *
 * This function writes the RX calibration tables for the specified band to
 * the EEPROM.
 *
 * @param [in] iBand
 *    GSM band (0:GSM-850, 1:GSM-900, 2:DCS-1800, 3:PCS-1900).
 *
 * @param [in] iUplink
 *    Uplink flag (0:downlink, X:downlink).
 *
 * @param [in] pRxCal
 *    Pointer to a RX calibration table structure.
 *
 * @return
 *    0 if or an error core.
 *
 ****************************************************************************/
eeprom_Error_t eeprom_WriteRxCal( int iBand, int iUplink, const eeprom_RxCal_t *pRxCal )
{
    int i;
    int err;
    int size;
    int nArfcn;
    eeprom_Cfg_t ee;
    eeprom_SID_t sId;
    eeprom_CfgRxCal_t *pCfgRxCal = NULL;

    // Get a copy of the EEPROM header
    err = eeprom_read( EEPROM_CFG_START_ADDR, sizeof(ee.hdr), (char *) &ee.hdr );
    if ( err != sizeof(ee.hdr) )
    {
#ifdef DISP_ERROR
        fprintf( stderr, "Error while reading the EEPROM content (%d)\n", err );
#endif
        return EEPROM_ERR_DEVICE;
    }

    // Validate the header magic ID
    if ( ee.hdr.u32MagicId != EEPROM_CFG_MAGIC_ID )
    {
        // Init the header
        ee.hdr.u32MagicId = EEPROM_CFG_MAGIC_ID;
        ee.hdr.u16Version = 1;

        // Write it to the EEPROM
        err = eeprom_write( EEPROM_CFG_START_ADDR, sizeof(ee.hdr) + sizeof(ee.cfg.v1), (const char *) &ee );
        if ( err != sizeof(ee.hdr) + sizeof(ee.cfg.v1) )
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Error while writing to the EEPROM (%d)\n", err );
#endif
            return EEPROM_ERR_DEVICE;
        }
    }

    switch ( ee.hdr.u16Version )
    {
        case 1:
        {
            int32_t fixVal;
    
            switch ( iBand )
            {
                case 0:
                    nArfcn = 124;
                    if ( iUplink )
                    {
                        sId = EEPROM_SID_GSM850_RXUCAL;
                        pCfgRxCal = &ee.cfg.v1.gsm850RxuCal;
                        size = sizeof(ee.cfg.v1.gsm850RxuCal) + sizeof(ee.cfg.v1.__gsm850RxuCalMem);
                    }
                    else
                    {
                        sId = EEPROM_SID_GSM850_RXDCAL;
                        pCfgRxCal = &ee.cfg.v1.gsm850RxdCal;
                        size = sizeof(ee.cfg.v1.gsm850RxdCal) + sizeof(ee.cfg.v1.__gsm850RxdCalMem);
                    }
                    break;
                case 1:
                    nArfcn = 194;
                    if ( iUplink )
                    {
                        sId = EEPROM_SID_GSM900_RXUCAL;
                        pCfgRxCal = &ee.cfg.v1.gsm900RxuCal;
                        size = sizeof(ee.cfg.v1.gsm900RxuCal) + sizeof(ee.cfg.v1.__gsm900RxuCalMem);
                    }
                    else
                    {
                        sId = EEPROM_SID_GSM900_RXDCAL;
                        pCfgRxCal = &ee.cfg.v1.gsm900RxdCal;
                        size = sizeof(ee.cfg.v1.gsm900RxdCal) + sizeof(ee.cfg.v1.__gsm900RxdCalMem);
                    }
                    break;
                case 2:
                    nArfcn = 374;
                    if ( iUplink )
                    {
                        sId = EEPROM_SID_DCS1800_RXUCAL;
                        pCfgRxCal = &ee.cfg.v1.dcs1800RxuCal;
                        size = sizeof(ee.cfg.v1.dcs1800RxuCal) + sizeof(ee.cfg.v1.__dcs1800RxuCalMem);
                    }
                    else
                    {
                        sId = EEPROM_SID_DCS1800_RXDCAL;
                        pCfgRxCal = &ee.cfg.v1.dcs1800RxdCal;
                        size = sizeof(ee.cfg.v1.dcs1800RxdCal) + sizeof(ee.cfg.v1.__dcs1800RxdCalMem);
                    }
                    break;
                case 3:
                    nArfcn = 299;
                    if ( iUplink )
                    {
                        sId = EEPROM_SID_PCS1900_RXUCAL;
                        pCfgRxCal = &ee.cfg.v1.pcs1900RxuCal;
                        size = sizeof(ee.cfg.v1.pcs1900RxuCal) + sizeof(ee.cfg.v1.__pcs1900RxuCalMem);
                    }
                    else
                    {
                        sId = EEPROM_SID_PCS1900_RXDCAL;
                        pCfgRxCal = &ee.cfg.v1.pcs1900RxdCal;
                        size = sizeof(ee.cfg.v1.pcs1900RxdCal) + sizeof(ee.cfg.v1.__pcs1900RxdCalMem);
                    }
                    break;
                default:
#ifdef DISP_ERROR
                    fprintf( stderr, "Invalid GSM band specified (%d)\n", iBand );
#endif
                    return EEPROM_ERR_INVALID;
            }

            pCfgRxCal->u16SectionID = sId;
            pCfgRxCal->u16Crc       = 0;
            pCfgRxCal->u32Time      = time(NULL);

            // Compress the IQ imbalance mode (0:off, 1:on, 2:auto)
            pCfgRxCal->u16IqImbalMode = pRxCal->u8IqImbalMode;

            // Compress the IQ imbalance compensation
            for ( i = 0; i < 4; i++ )
            {
                pCfgRxCal->u16IqImbalCorr[i] = pRxCal->u16IqImbalCorr[i];
            }

            // Compress the External RX gain
            fixVal = (int32_t)(pRxCal->fExtRxGain * 512.f + (pRxCal->fExtRxGain>0 ? 0.5f:-0.5f));
                             if ( fixVal >  32767 ) pCfgRxCal->sfixExtRxGain =  32767;
                        else if ( fixVal < -32768 ) pCfgRxCal->sfixExtRxGain = -32768;
                        else                        pCfgRxCal->sfixExtRxGain = (int16_t)fixVal;

            // Compress the Mixer gain error compensation
            fixVal = (int32_t)(pRxCal->fRxMixGainCorr * 512.f + (pRxCal->fRxMixGainCorr>0 ? 0.5f:-0.5f));
                 if ( fixVal >  32767 ) pCfgRxCal->sfixRxMixGainCorr =  32767;
            else if ( fixVal < -32768 ) pCfgRxCal->sfixRxMixGainCorr = -32768;
            else                        pCfgRxCal->sfixRxMixGainCorr = (int16_t)fixVal;

            // Compress the LNA gain error compensation (1:@-12 dB, 2:@-24 dB, 3:@-36 dB)
            for ( i = 0; i < 3; i++ )
            {
                fixVal = (int32_t)(pRxCal->fRxLnaGainCorr[i] * 512.f + (pRxCal->fRxLnaGainCorr[i]>0 ? 0.5f:-0.5f));
                     if ( fixVal >  32767 ) pCfgRxCal->sfixRxLnaGainCorr[i] =  32767;
                else if ( fixVal < -32768 ) pCfgRxCal->sfixRxLnaGainCorr[i] = -32768;
                else                        pCfgRxCal->sfixRxLnaGainCorr[i] = (int16_t)fixVal;
            }

            // Compress the Frequency roll-off compensation
            for ( i = 0; i < nArfcn; i++ )
            {
                fixVal = (int32_t)(pRxCal->fRxRollOffCorr[i] * 512.f + (pRxCal->fRxRollOffCorr[i]>0 ? 0.5f:-0.5f));
                     if ( fixVal >  32767 ) pCfgRxCal->sfixRxRollOffCorr[i] =  32767;
                else if ( fixVal < -32768 ) pCfgRxCal->sfixRxRollOffCorr[i] = -32768;
                else                        pCfgRxCal->sfixRxRollOffCorr[i] = (int16_t)fixVal;
            }

            // Add the CRC
            pCfgRxCal->u16Crc = eeprom_crc( (uint8_t *)&pCfgRxCal->u32Time, size - 2 * sizeof(uint16_t) );

            // Write it to the EEPROM
            err = eeprom_write( EEPROM_CFG_START_ADDR + ((uint32_t)pCfgRxCal - (uint32_t)&ee), size, (const char *)pCfgRxCal );
            if ( err != size )
            {
#ifdef DISP_ERROR
                fprintf( stderr, "Error while writing to the EEPROM (%d)\n", err );
#endif
                return EEPROM_ERR_DEVICE;
            }
            break;
        }

        default:
        {
#ifdef DISP_ERROR
            fprintf( stderr, "Unsupported header version\n" );
#endif
            return EEPROM_ERR_UNSUPPORTED;
        }
    }
    return EEPROM_SUCCESS;
}


/****************************************************************************
 *                            Private functions                             *
 ****************************************************************************/

/**
 * Dump the content of the EEPROM to the standard output
 */
int eeprom_dump( int addr, int size, int hex )
{
    FILE *f;
    char ch;
    int i;
    
    f = fopen( EEPROM_DEV, "r+" );
    if ( f == NULL )
    {
        perror( "eeprom fopen" );
        return -1;
    }
    fseek( f, addr, SEEK_SET );
    
    for ( i = 0; i < size; ++i, ++addr )
    {
        if ( fread( &ch, 1, 1, f ) != 1 )
        {
            perror( "eeprom fread" );
            fclose( f );
            return -1;
        }
        if ( hex )
        {
            if ( (i % 16) == 0 )
            {
                printf( "\n %.4x|  ", addr );
            }
            else if ( (i % 8) == 0 )
            {
                printf( "  " );
            }
            printf( "%.2x ", ch );
        }
        else
            putchar( ch );
    }
    if ( hex )
    {
        printf( "\n\n" );
    }
    fflush( stdout );
    
    fclose( f );
    return 0;
}

static FILE *g_file;

/**
 * Read up to 'size' bytes of data from the EEPROM starting at offset 'addr'.
 */
static int eeprom_read( int addr, int size, char *pBuff )
{
    FILE *f = g_file;
    int n;
    
    if (!f) {
    	f = fopen( EEPROM_DEV, "r+" );
    	if ( f == NULL )
        {
            perror( "eeprom fopen" );
            return -1;
        }
	g_file = f;
    }
    fseek( f, addr, SEEK_SET );
    n = fread( pBuff, 1, size, f );
    return n;
}

    
/**
 * Write up to 'size' bytes of data to the EEPROM starting at offset 'addr'.
 */
static int eeprom_write( int addr, int size, const char *pBuff )
{
    FILE *f = g_file;
    int n;

    if (!f) {
        f = fopen( EEPROM_DEV, "r+" );
        if ( f == NULL )
        {
            perror( "eeprom fopen" );
            return -1;
        }
	g_file = f;
    }
    fseek( f, addr, SEEK_SET );
    n = fwrite( pBuff, 1, size, f );
    fclose( f );
    return n;
}


/**
 * EEPROM CRC.
 */
static uint16_t eeprom_crc( uint8_t *pu8Data, int len )
{
    int i;
    uint16_t crc = 0xFFFF;

    while (len--) {
        crc ^= (uint16_t)*pu8Data++;

        for (i=0; i<8; i++) {
            if (crc & 1) crc = (crc >> 1) ^ 0x8408;
            else         crc = (crc >> 1);
        }
    }

    crc = ~crc;
    return crc;
}
