/*
 * eCan.c
 *
 *  Created on: Oct 1, 2018
 *      Author: esr
 */
//###########################################################################
//
// FILE:    F2806x_ECan.c
//
// TITLE:   F2806x Enhanced CAN Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2806x Support Library v2.03.00.00 $
// $Release Date: Sun Mar 25 13:24:47 CDT 2018 $
// $Copyright:
// Copyright (C) 2009-2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include "F2806x_Device.h"     // F2806x Headerfile Include File
#include "F2806x_Examples.h"   // F2806x Examples Include File
#include <stdint.h>
#include "include/traveSM.h"
//
// InitECan - This function initializes the eCAN module to a known state.
//
#if DSP28_ECANA

void ECan_sendMBox(uint16_t mbox_n);
//
// InitECana - Initialize eCAN-A module
//
void ECan_init(void)
{
    //
    struct ECAN_REGS ECanaShadow;
    // Setup variables to initialize mailboxes to zero
    //
    int16 mboxCount;
    volatile struct MBOX *Mailbox = (void *) 0x6100;

    EALLOW;
    // EALLOW enables access to protected bits

    //
    // Configure eCAN RX and TX pins for CAN operation using eCAN regs
    //
    ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
    ECanaShadow.CANTIOC.bit.TXFUNC = 1;
    ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

    ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
    ECanaShadow.CANRIOC.bit.RXFUNC = 1;
    ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;

    //
    // Configure eCAN for HECC mode - (reqd to access mailboxes 16 thru 31)
    // HECC mode also enables time-stamping feature
    //
    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.SCB = 1;
    ECanaShadow.CANMC.bit.STM = 0;    // Configure CAN for self-test mode if 1

    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;
    //
    // Initialize all registers of the mailboxes to zero
    for (mboxCount = 0; mboxCount < 32; mboxCount++)
    {
        Mailbox->MSGID.all = 0;
        Mailbox->MSGCTRL.all = 0;
        Mailbox->MDH.all = 0;
        Mailbox->MDL.all = 0;
        Mailbox = Mailbox + 1;
    }

    //
    // TAn, RMPn, GIFn bits are all zero upon reset and are cleared again
    //  as a matter of precaution.
    //
    ECanaRegs.CANTA.all = 0xFFFFFFFF;    // Clear all TAn bits

    ECanaRegs.CANRMP.all = 0xFFFFFFFF;   // Clear all RMPn bits

    ECanaRegs.CANGIF0.all = 0xFFFFFFFF;  // Clear all interrupt flag bits
    ECanaRegs.CANGIF1.all = 0xFFFFFFFF;

    //
    // Configure bit timing parameters for eCANA
    //
    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 1;            // Set CCR = 1
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    //
    // Wait until the CPU has been granted permission to change the
    // configuration registers
    //
    do
    {
        ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    }
    while (ECanaShadow.CANES.bit.CCE != 1);   // Wait for CCE bit to be set

    ECanaShadow.CANBTC.all = 0;

    //
    // The following block is for 90 MHz SYSCLKOUT.
    // (45 MHz CAN module clock Bit rate = 1 Mbps)
    // See Note at end of file.
    //
    // http://www.bittiming.can-wiki.info/   BT=15
    // 90 MHz SYSCLKOUT. (45 MHz CAN module clock)
    ECanaShadow.CANBTC.all = 0x00040061;
;
    //ECanaShadow.CANBTC.bit.BRPREG = 3;
    //ECanaShadow.CANBTC.bit.TSEG2REG = 2;
    //ECanaShadow.CANBTC.bit.TSEG1REG = 12;

    ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

    ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
    ECanaShadow.CANMC.bit.CCR = 0;            // Set CCR = 0
    ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

    //
    // Wait until the CPU no longer has permission to change the
    // configuration registers
    //
    do
    {
        ECanaShadow.CANES.all = ECanaRegs.CANES.all;
    }
    while (ECanaShadow.CANES.bit.CCE != 0);   // Wait for CCE bit to be  cleared

    //
    //  Disable all Mailboxes
    //
    ECanaRegs.CANME.all = 0;        // Required before writing the MSGIDs

    EDIS;
}


void ECan_SetupMbox(void)
{
    struct ECAN_REGS ECanaShadow;
    ECanaShadow = ECanaRegs;
    volatile struct MBOX *Mailbox_p;
    //
    // Mailboxs can be written to 16-bits or 32-bits at a time
    // Write to the MSGID field of TRANSMIT mailboxes MBOX0 - 15
    //
    Mailbox_p = MBOX_P(MSG_CTRL_MBOX);
    Mailbox_p->MSGID.all = MSG_CTRL_ID;
    Mailbox_p->MSGID.bit.IDE = 1;
    Mailbox_p->MSGCTRL.bit.DLC = MSG_CTRL_LEN;

    Mailbox_p = MBOX_P(MSG_STATUS_MBOX);
    Mailbox_p->MSGID.all = MSG_STATUS_ID;
    Mailbox_p->MSGID.bit.IDE = 1;
    Mailbox_p->MSGCTRL.bit.DLC = MSG_STATUS_LEN;
    /*
    Mailbox_p = MBOX_P(MSG_SR50_MBOX);
    Mailbox_p->MSGID.all = MSG_SR50_ID;
    Mailbox_p->MSGID.bit.IDE = 1;
    Mailbox_p->MSGCTRL.bit.DLC = MSG_SR50_LEN;
    */
    Mailbox_p = MBOX_P(MSG_SR100_MBOX);
    Mailbox_p->MSGID.all = MSG_SR100_ID;
    Mailbox_p->MSGID.bit.IDE = 1;
    Mailbox_p->MSGCTRL.bit.DLC = MSG_SR100_LEN;
    /*
    Mailbox_p = MBOX_P(MSG_SR200_MBOX);
    Mailbox_p->MSGID.all = MSG_SR200_ID;
    Mailbox_p->MSGID.bit.IDE = 1;
    Mailbox_p->MSGCTRL.bit.DLC = MSG_SR200_LEN;
    */
    Mailbox_p = MBOX_P(MSG_SR400_MBOX);
    Mailbox_p->MSGID.all = MSG_SR400_ID;
    Mailbox_p->MSGID.bit.IDE = 1;
    Mailbox_p->MSGCTRL.bit.DLC = MSG_SR400_LEN;

    Mailbox_p = MBOX_P(MSG_MAVG_MBOX);
    Mailbox_p->MSGID.all = MSG_MAVG_ID;
    Mailbox_p->MSGID.bit.IDE = 1;
    Mailbox_p->MSGCTRL.bit.DLC = MSG_MAVG_LEN;

    Mailbox_p = MBOX_P(MSG_MSD_MBOX);
    Mailbox_p->MSGID.all = MSG_MSD_ID;
    Mailbox_p->MSGID.bit.IDE = 1;
    Mailbox_p->MSGCTRL.bit.DLC = MSG_MSD_LEN;

    //
    // Configure Mailboxes 0,1 as Tx, 2 as Rx
    ECanaShadow.CANMD.all = 0;
    ECanaShadow.CANMD.bit.MD0 = 1;
    ECanaShadow.CANMD.bit.MD1 = 0;
    ECanaShadow.CANMD.bit.MD2 = 0;
    ECanaShadow.CANMD.bit.MD3 = 0;
    ECanaShadow.CANMD.bit.MD4 = 0;
    ECanaShadow.CANMD.bit.MD5 = 0;
    ECanaShadow.CANMD.bit.MD6 = 0;
    ECanaShadow.CANMD.bit.MD7 = 0;

    ECanaRegs.CANMD.all = ECanaShadow.CANMD.all;
    //
    // Enable Mailboxes
    ECanaShadow.CANME.all = 0x00000000;
    ECanaShadow.CANME.bit.ME0 = 1;
    ECanaShadow.CANME.bit.ME1 = 1;
    ECanaShadow.CANME.bit.ME2 = 1;
    ECanaShadow.CANME.bit.ME3 = 1;
    ECanaShadow.CANME.bit.ME4 = 1;
    ECanaShadow.CANME.bit.ME5 = 1;
    ECanaShadow.CANME.bit.ME6 = 1;
    ECanaShadow.CANME.bit.ME7 = 1;
    ECanaRegs.CANME.all = ECanaShadow.CANME.all;
    //
    // Since this write is to the entire register
    EALLOW;
    ECanaRegs.CANMIM.all = 0x00000000;
    EDIS;
}


void ECan_setMBoxData(uint16_t mbox_n, uint32_t tx_data[])
{
    volatile struct MBOX *Mailbox;
    Mailbox = MBOX_P(mbox_n);
    Mailbox->MDL.all = tx_data[0];  //
    Mailbox->MDH.all = tx_data[1];  //
}
// mailbox_read - This function reads out the contents of the indicated
// by the Mailbox number (MBXnbr). MSGID of a rcv MBX is transmitted as the
// MDL data.
//
uint16_t ECan_getCTRLMBoxData(msg_ctrl_t* msg_p)
{
    volatile struct MBOX *Mailbox_p = MBOX_P(MSG_CTRL_MBOX);
    //clear flag
    EALLOW;
    ECanaRegs.CANRMP.all=(1<<MSG_CTRL_MBOX);
    EDIS;
    msg_p->data = ((uint32_t)Mailbox_p->MDL.word.LOW_WORD<<16) | Mailbox_p->MDH.word.HI_WORD ;  //
    msg_p->ctrl.all = Mailbox_p->MDL.word.HI_WORD;  //
    if (ECanaRegs.CANRMP.bit.RMP4==0) {
        return 0;
    }else{
        return 1;
    };
}
void ECan_sendNotifyMBox(msg_notify_t* msg_p)
{
    volatile struct MBOX *Mailbox_p = MBOX_P(MSG_STATUS_MBOX);
    //clear flag
    Mailbox_p->MDL.all=msg_p->header.all;//
    Mailbox_p->MDH.all=msg_p->data;  //
    ECan_sendMBox(MSG_STATUS_MBOX);
}


uint16_t ECan_sendMBox_Blocking(uint16_t mbox_n)
{
#define N_LOOPS 2000000
    uint32_t i = 0;
    uint32_t mask = 0x00000001;
    uint16_t ret = 1;
    mask = mask << mbox_n;
    ECanaRegs.CANTRS.all |= mask;
    // Wait for TAn bits to be set
    while ((ECanaRegs.CANTA.all & mask) != 1)
    {
        i++;
        if (i > N_LOOPS)
        {
            ECanaRegs.CANTRS.all &= ~mask;
            ret = 0;
            break;
        }
    }
    // Wait for TRS bits to 0
    while ((ECanaRegs.CANTRS.all & mask) != 0)
        ;
    //ECanaRegs.CANTA.all |= mask;
    // Wait for all TAn bits to be set
    //while ((ECanaRegs.CANTA.all & mask) != 1);
    return ret;
}
void ECan_sendMBox(uint16_t mbox_n)
{
    uint32_t mask = 0x00000001;
    mask = mask << mbox_n;
    ECanaRegs.CANTRS.all |= mask;
}

#endif // endif DSP28_ECANA

//
// Note: Bit timing parameters must be chosen based on the network parameters
// such as the sampling point desired and the propagation delay of the network.
// The propagation delay is a function of length of the cable, delay introduced
// by the transceivers and opto/galvanic-isolators (if any).
//
// The parameters used in this file must be changed taking into account the
// above mentioned factors in order to arrive at the bit-timing parameters
// suitable for a network.
//

//
// End of file
//



