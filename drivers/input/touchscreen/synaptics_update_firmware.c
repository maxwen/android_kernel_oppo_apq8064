/************************************************************************************
** drivers/input/touchscreen/synaptics_update_firmware.c
**
** Copyright (C), 2011-2012, OPPO Mobile Comm Corp., Ltd
** All rights reserved.
** 
** VENDOR_EDIT
** 
** from

   Copyright (c) 2011 Synaptics, Inc.

   Permission is hereby granted, free of charge, to any person obtaining a copy of
   this software and associated documentation files (the "Software"), to deal in
   the Software without restriction, including without limitation the rights to use,
   copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
   Software, and to permit persons to whom the Software is furnished to do so,
   subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.

** --------------------------- Revision History: --------------------------------
** <author>		                      <data> 	<version >  <desc>
** ------------------------------------------------------------------------------
** LiuJun@OnlineRD.Driver.TouchScreen  2012/11/14   1.0	    create file
** ------------------------------------------------------------------------------
** 
*/

// SynaFirmwareImage.h contains the data for both the entire image and the config block

#include <linux/i2c.h>
#include <linux/delay.h>

static struct i2c_client *tp_client;

static int readRMI(unsigned short regAddress, unsigned char* buffer, int len)
{
    i2c_smbus_read_i2c_block_data(tp_client, regAddress,
                  len,  buffer);
    return 0;
}

static int writeRMI(unsigned short regAddress, const unsigned char* buffer, int len)
{
    i2c_smbus_write_i2c_block_data(tp_client, regAddress,
                  len, buffer);
    return 0;
}

/* Variables for F34 functionality */
unsigned short SynaF34DataBase;
unsigned short SynaF34QueryBase;
unsigned short SynaF01DataBase;
unsigned short SynaF01CommandBase;

unsigned short SynaF34Reflash_BlockNum;
unsigned short SynaF34Reflash_BlockData;
unsigned short SynaF34ReflashQuery_BootID;
unsigned short SynaF34ReflashQuery_FlashPropertyQuery;
unsigned short SynaF34ReflashQuery_FirmwareBlockSize;
unsigned short SynaF34ReflashQuery_FirmwareBlockCount;
unsigned short SynaF34ReflashQuery_ConfigBlockSize;
unsigned short SynaF34ReflashQuery_ConfigBlockCount;

unsigned short SynaFirmwareBlockSize;
unsigned short SynaFirmwareBlockCount;
unsigned long  SynaImageSize;

unsigned short SynaConfigBlockSize;
unsigned short SynaConfigBlockCount;
unsigned long SynaConfigImageSize;

unsigned short SynaBootloadID;

unsigned short SynaF34_FlashControl;

const unsigned char *SynafirmwareImgData;
const unsigned char *SynaconfigImgData;
const unsigned char *Syna_Firmware_File_Data;

//unsigned char *SynalockImgData;
//unsigned int SynafirmwareImgVersion;

/* End: Variables for F34 functionality */

/* SynaSetup scans the Page Description Table (PDT) and sets up the necessary variables
 * for the reflash process. This function is a "slim" version of the PDT scan function in
 * in PDT.c, since only F34 and F01 are needed for reflash.
 */
static void SynaSetup(void)
{
    unsigned char address;
    unsigned char buffer[6];
    printk("\nSynaSetup... ");
    for (address = 0xe9; address > 0xdc/*0xc0*/; address = address - 6) {
        readRMI(address, buffer, 6);
        //printk("----Finding fn base--buffer[5]=0x%02x---\n",buffer[5]);
        switch (buffer[5]) {
        case 0x34:
            SynaF34DataBase = buffer[3];
            SynaF34QueryBase = buffer[0];
            break;
        case 0x01:
            SynaF01DataBase = buffer[3];
            SynaF01CommandBase = buffer[1];
            break;
        }
    }
    printk("--[F34] data=0x%02x query=0x%02x [F01] data=0x%02x cmd=0x%02x--\n",
        SynaF34DataBase, SynaF34QueryBase, SynaF01DataBase, SynaF01CommandBase);

    SynaF34Reflash_BlockNum = SynaF34DataBase;
    SynaF34Reflash_BlockData = SynaF34DataBase + 2;
    SynaF34ReflashQuery_BootID = SynaF34QueryBase;
    SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 2;
    SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 3;
    SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +5;
    SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
    SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 7;

    SynaF34_FlashControl = SynaF34DataBase + SynaFirmwareBlockSize + 2;

#if 0
    SynafirmwareImgVersion = (unsigned int)(SynaFirmware[7]);

    switch (SynafirmwareImgVersion) {
    case 2:
        SynalockImgData = (unsigned char *)((&SynaFirmware[0]) + 0xD0);
        break;
    case 3:
        SynalockImgData = (unsigned char *)((&SynaFirmware[0]) + 0xC0);
        break;
    default:
        break;
    }
#endif
}

/* SynaInitialize sets up the reflahs process
 */
static void SynaInitialize(void)
{
    unsigned char uData[1]; // uData[2]
    unsigned char uStatus;

    printk("\nInitializing Reflash Process...");
#define PAGE_SELECT_REG 0xff
    uData[0] = 0x00;
    writeRMI(PAGE_SELECT_REG, uData, 1); //select page 0

    SynafirmwareImgData = 0;

    SynaconfigImgData = 0 ;

    do {
        readRMI(0, &uStatus, 1);

        if (uStatus & 0x80) {
            break;
        }
    } while (uStatus & 0x40);

    SynaSetup();

    //readRMI(SynaF34ReflashQuery_FirmwareBlockSize, &uData[0], 2);

    //SynaFirmwareBlockSize = uData[0] | (uData[1] << 8);
}

/* SynaReadFirmwareInfo reads the F34 query registers and retrieves the block size and count
 * of the firmware section of the image to be reflashed
 */
static void SynaReadFirmwareInfo(void)
{
    unsigned char uData[2];

    printk("\nRead Firmware Info");

    readRMI(SynaF34ReflashQuery_FirmwareBlockSize, &uData[0], 2);
    SynaFirmwareBlockSize = uData[0] | (uData[1] << 8);

    readRMI(SynaF34ReflashQuery_FirmwareBlockCount, &uData[0], 2);
    SynaFirmwareBlockCount = uData[0] | (uData[1] << 8);
    SynaImageSize = SynaFirmwareBlockCount * SynaFirmwareBlockSize;

    printk("\nFirmware block:%d count:%d  total_size:%lu",
            SynaFirmwareBlockSize, SynaFirmwareBlockCount, SynaImageSize);
}

/* SynaReadConfigInfo reads the F34 query registers and retrieves the block size and count
 * of the configuration section of the image to be reflashed
 */
static void SynaReadConfigInfo(void)
{
    unsigned char uData[2];

    printk("\nRead Config Info");

    readRMI(SynaF34ReflashQuery_ConfigBlockSize, &uData[0], 2);
    SynaConfigBlockSize = uData[0] | (uData[1] << 8);

    readRMI(SynaF34ReflashQuery_ConfigBlockCount, &uData[0], 2);
    SynaConfigBlockCount = uData[0] | (uData[1] << 8);
    SynaConfigImageSize = SynaConfigBlockCount * SynaConfigBlockSize;

    printk("\nConfig   block:%d count:%d  total_size:%lu",
            SynaConfigBlockSize, SynaConfigBlockCount, SynaConfigImageSize);
}


/* SynaReadBootloadID reads the F34 query registers and retrieves the bootloader ID of the firmware
 */
static void SynaReadBootloadID(void)
{
    unsigned char uData[2];

    readRMI(SynaF34ReflashQuery_BootID, &uData[0], 2);
    SynaBootloadID = uData[0] + uData[1] * 0x100;
}

/* SynaWriteBootloadID writes the bootloader ID to the F34 data register to unlock the reflash process
 */
static void SynaWriteBootloadID(void)
{
    unsigned char uData[2];

    uData[0] = SynaBootloadID % 0x100;
    uData[1] = SynaBootloadID / 0x100;

    writeRMI(SynaF34Reflash_BlockData, &uData[0], 2);
}

/* SynaWaitForATTN waits for ATTN to be asserted within a certain time threshold.
 */
static void SynaWaitForATTN(void)
{
//    unsigned int error;

    //error = waitATTN(ASSERT, 300);
    printk("----wait for ATTN 800 ms---\n");
    msleep(800);
}

/* SynaWaitATTN waits for ATTN to be asserted within a certain time threshold.
 * The function also checks for the F34 "Program Enabled" bit and clear ATTN accordingly.
 */
static void SynaWaitATTN(void)
{
    unsigned char uData;
    unsigned char uStatus;

    //waitATTN(ASSERT, 300);
    msleep(800);

    do {
        readRMI(SynaF34_FlashControl, &uData, 1);
        //printk("----SynaWaitATTN--1\n");
        readRMI((SynaF01DataBase + 1), &uStatus, 1);
        //printk("----SynaWaitATTN--2\n");
    } while (uData!= 0x80);
}

/* SynaEnableFlashing kicks off the reflash process
 */
static int SynaEnableFlashing(void)
{
    unsigned char uData;
    unsigned char uStatus;
    int retry = 3;

    printk("\nEnable Reflash...\n");

    // Reflash is enabled by first reading the bootloader ID from the firmware and write it back
    SynaReadBootloadID();
    SynaWriteBootloadID();

    // Make sure Reflash is not already enabled
    do {
        readRMI(SynaF34_FlashControl, &uData, 1);
        printk("----Read reflash enable ---uData=0x%x--\n",uData);
    } while (uData  ==  0x0f);//while (((uData & 0x0f) != 0x00));

    // Clear ATTN
    readRMI (SynaF01DataBase, &uStatus, 1);
    printk("----Read status ---uStatus=0x%x--\n",uStatus);
    if ((uStatus &0x40) == 0) {
        // Write the "Enable Flash Programming command to F34 Control register
        // Wait for ATTN and then clear the ATTN.
        //uData = 0x0f;    //lemon
        readRMI(SynaF34_FlashControl, &uData, 1);
        uData = uData | 0x0f;  
        writeRMI(SynaF34_FlashControl, &uData, 1);
        SynaWaitForATTN();
        readRMI((SynaF01DataBase + 1), &uStatus, 1);

        // Scan the PDT again to ensure all register offsets are correct
        SynaSetup();

        // Read the "Program Enabled" bit of the F34 Control register, and proceed only if the
        // bit is set.
        readRMI(SynaF34_FlashControl, &uData, 1);
        printk("----read--enable ---uData=0x%x--\n",uData);
        while (uData != 0x80) {
            // In practice, if uData!=0x80 happens for multiple counts, it indicates reflash
            // is failed to be enabled, and program should quit
            printk("%s Can NOT enable reflash !!!\n",__func__);

            if (!retry--)
                return -1;
            readRMI(SynaF34_FlashControl, &uData, 1);
            printk("----read--enable ---uData=0x%x--\n",uData);
        }
    }
    return 0;
}

static void SynaWaitATTN1(void)
{
    unsigned char uData;
    unsigned char uStatus;

    //waitATTN(ASSERT, 300);
    msleep(5);

    do {
        readRMI(SynaF34_FlashControl, &uData, 1);
        //printk("----SynaWaitATTN1-1-1\n");
        readRMI((SynaF01DataBase + 1), &uStatus, 1);
        //printk("----SynaWaitATTN1-1-2\n");
    } while (uData!= 0x80);
}
/* SynaProgramConfiguration writes the configuration section of the image block by block
 */
static void SynaProgramConfiguration(void)
{
    unsigned char uData[2];
    const unsigned char *puData = SynaconfigImgData; //ConfigBlockData

    unsigned short blockNum;
   // int i;
    printk("\nProgram Configuration Section...\n");

    for (blockNum = 0; blockNum < SynaConfigBlockCount; blockNum++) {
        uData[0] = blockNum & 0xff;
        uData[1] = (blockNum & 0xff00) >> 8;

        printk("--Writing config-- block: %d/%d \n", blockNum+1, SynaConfigBlockCount);
        //Block by blcok, write the block number and data to the corresponding F34 data registers
        writeRMI(SynaF34Reflash_BlockNum, &uData[0], 2);
        writeRMI(SynaF34Reflash_BlockData, puData, SynaConfigBlockSize);
        puData += SynaConfigBlockSize;

        // Issue the "Write Configuration Block" command
        uData[0] = 0x06;
        writeRMI(SynaF34_FlashControl, &uData[0], 1);
        SynaWaitATTN();
        //printk(".");
    }
}

/* SynaFinalizeReflash finalizes the reflash process
 */
static void SynaFinalizeReflash(void)
{
    unsigned char uData;
    unsigned char uStatus;

    printk("\nFinalizing Reflash...\n");

    // Issue the "Reset" command to F01 command register to reset the chip
    // This command will also test the new firmware image and check if its is valid
    uData = 1;
    writeRMI(SynaF01CommandBase, &uData, 1);

    SynaWaitForATTN();
    readRMI(SynaF01DataBase, &uData, 1);
    printk("-----SynaFinalizeReflash-1-\n");
    // Sanity check that the reflash process is still enabled
    do {
        readRMI(SynaF34_FlashControl, &uStatus, 1);
        printk("-----SynaFinalizeReflash-2-\n");
    } while ((uStatus & 0x0f) != 0x00);
    printk("-----SynaFinalizeReflash-3-\n");
    readRMI((SynaF01DataBase + 1), &uStatus, 1);
    printk("-----SynaFinalizeReflash-4-\n");
    SynaSetup();
    printk("-----SynaFinalizeReflash-5-\n");
    uData = 0;

    // Check if the "Program Enabled" bit in F01 data register is cleared
    // Reflash is completed, and the image passes testing when the bit is cleared
    do {
        readRMI(SynaF01DataBase, &uData, 1);
        printk("-----SynaFinalizeReflash-6- data=%02x\n", uData);
    } while ((uData & 0x40) != 0);
    printk("-----SynaFinalizeReflash-7-\n");
    // Rescan PDT the update any changed register offsets
    SynaSetup();

    printk("\nReflash Completed. Please reboot.\n");
}

/* SynaFlashFirmwareWrite writes the firmware section of the image block by block
 */
static void SynaFlashFirmwareWrite(void)
{
    const unsigned char *puFirmwareData = SynafirmwareImgData;
    unsigned char uData[2];
    unsigned short  blockNum;
    printk("----SynaFlashFirmwareWrite----\n");
    for (blockNum = 0; blockNum < SynaFirmwareBlockCount; ++blockNum) {
        //Block by blcok, write the block number and data to the corresponding F34 data registers
        uData[0] = blockNum & 0xff;
        uData[1] = (blockNum & 0xff00) >> 8;
        printk("--Writing data-- block: %d/%d \n", blockNum+1, SynaFirmwareBlockCount);
        writeRMI(SynaF34Reflash_BlockNum, &uData[0], 2);
        //printk("--SynaFlashFirmwareWrite----2\n");
        writeRMI(SynaF34Reflash_BlockData, puFirmwareData, SynaFirmwareBlockSize);
        puFirmwareData += SynaFirmwareBlockSize;
        //printk("--SynaFlashFirmwareWrite----3\n");
        // Issue the "Write Firmware Block" command
        uData[0] = 2;
        writeRMI(SynaF34_FlashControl, &uData[0], 1);
        //printk("--SynaFlashFirmwareWrite----4\n");
        SynaWaitATTN1();
    }

}

/* SynaProgramFirmware prepares the firmware writing process
 */
static void SynaProgramFirmware(void)
{
    unsigned char uData;

    printk("\nProgram Firmware Section...");

    SynaReadBootloadID();
    printk("\n------------SynaReadBootloadID()--");
    SynaWriteBootloadID();
    printk("\n-------------SynaWriteBootloadID()--");
    uData = 3;
    writeRMI(SynaF34_FlashControl, &uData, 1);
    printk("\n------------writeRMI(SynaF34_FlashControl, &uData, 1)--\n");
    msleep(1000);
    SynaWaitATTN();
    printk("\n-------------SynaWaitATTN()---\n");
    SynaFlashFirmwareWrite();
    printk("\n-------------SynaFlashFirmwareWrite()---\n");
}

#if 0
/* SynaBootloaderLock locks down the bootloader
*/
static void SynaBootloaderLock(void)
{
    unsigned short lockBlockCount;
    unsigned char *puFirmwareData = SynalockImgData;
    unsigned char uData[2];
    unsigned short uBlockNum;

    // Check if device is in unlocked state
    readRMI((SynaF34QueryBase+ 2), &uData[0], 1);

    //Device is unlocked
    if (uData[0] & 0x02) {
        printk("Device unlocked. Lock it first...\n");
        // Different bootloader version has different block count for the lockdown data
        // Need to check the bootloader version from the image file being reflashed
        switch (SynafirmwareImgVersion) {
        case 2:
            lockBlockCount = 3;
            break;
        case 3:
            lockBlockCount = 4;
            break;
        default:
            lockBlockCount = 0;
            break;
        }

        // Write the lockdown info block by block
        // This reference code of lockdown process does not check for bootloader version
        // currently programmed on the ASIC against the bootloader version of the image to
        // be reflashed. Such case should not happen in practice. Reflashing cross different
        // bootloader versions is not supported.
        for (uBlockNum = 0; uBlockNum < lockBlockCount; ++uBlockNum) {
            uData[0] = uBlockNum & 0xff;
            uData[1] = (uBlockNum & 0xff00) >> 8;

            /* Write Block Number */
            readRMI(SynaF34Reflash_BlockNum, &uData[0], 2);

            /* Write Data Block */
            writeRMI(SynaF34Reflash_BlockData, puFirmwareData, SynaFirmwareBlockSize);

            /* Move to next data block */
            puFirmwareData += SynaFirmwareBlockSize;

            /* Issue Write Lockdown Block command */
            uData[0] = 4;
            writeRMI(SynaF34_FlashControl, &uData[0], 1);

            /* Wait ATTN until device is done writing the block and is ready for the next. */
            SynaWaitATTN();
        }
        printk("Device locking done.\n");

        // Enable reflash again to finish the lockdown process.
        // Since this lockdown process is part of the reflash process, we are enabling
        // reflash instead, rather than resetting the device to finish the unlock procedure.
        SynaEnableFlashing();
    } else printk("Device already locked.\n");
}

/* eraseConfigBlock erases the config block
*/
static void eraseConfigBlock(void)
{
    unsigned char uData;

    // Erase fo config block is done by first entering into bootloader mode
    SynaReadBootloadID();
    SynaWriteBootloadID();

    // Command 7 to erase config block
    uData = 7;
    writeRMI(SynaF34_FlashControl, &uData, 1);

    SynaWaitATTN();
}


/* ConfigBlockReflash reflashes the config block only
*/
static void ConfigBlockReflash(struct i2c_client *client)
{
    unsigned char uData[2];
    tp_client = client;
    SynaInitialize();
    printk("-----111--\n");
    SynaReadConfigInfo();
    printk("-----222--\n");
    SynaReadFirmwareInfo();
    printk("-----333--\n");
    SynaF34_FlashControl = SynaF34DataBase + SynaFirmwareBlockSize + 2;

    SynaEnableFlashing();
    printk("-----444--\n");
    // Check if device is in unlocked state
    readRMI((SynaF34QueryBase + 2), &uData[0], 1);
    printk("-----555--\n");
    //Device is unlocked
    if (uData[0] & 0x02) {
        printk("---------Device is unlocked----\n");
        SynaFinalizeReflash();
        return;
        // Do not reflash config block if not locked.
    }
    printk("-----666--\n");
    eraseConfigBlock();
    SynaconfigImgData = (unsigned char *)ConfigBlockData;
    printk("-----777--\n");
    SynaProgramConfiguration();
    printk("-----888--\n");
    SynaFinalizeReflash();
    printk("-----999--\n");
}
#endif


/* CompleteReflash reflashes the entire user image, including the configuration block and firmware
*/
void CompleteReflash(struct i2c_client *client, const unsigned char* firmware_data)
{
    int ret = 0;
    tp_client = client;
    Syna_Firmware_File_Data = firmware_data;

    SynaInitialize();

    SynaReadConfigInfo();

    SynaReadFirmwareInfo();

    SynafirmwareImgData = (unsigned char *)(Syna_Firmware_File_Data + 0x100);
    SynaconfigImgData  = (unsigned char *)(SynafirmwareImgData+SynaImageSize);

    SynaF34_FlashControl = SynaF34DataBase + SynaFirmwareBlockSize + 2;

    ret = SynaEnableFlashing();
    if (ret)
        return;

    SynaProgramFirmware();

    SynaProgramConfiguration();

    SynaFinalizeReflash();
}

