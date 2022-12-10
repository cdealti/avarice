/*
 *	avarice - The "avarice" program.
 *	Copyright (C) 2001 Scott Finneran
 *      Copyright (C) 2002 Intel Corporation
 *	Copyright (C) 2005, 2007 Joerg Wunsch
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License Version 2
 *      as published by the Free Software Foundation.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111, USA.
 *
 * This file implements target execution handling for the mkII protocol.
 *
 * $Id$
 */


#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

#include "avarice.h"
#include "jtag.h"
#include "jtag2.h"
#include "remote.h"

unsigned long jtag2::getProgramCounter(void)
{
    if (cached_pc_is_valid)
        return cached_pc;

    uchar *response;
    int responseSize;
    uchar command[] = { CMND_READ_PC };

    try
    {
        doJtagCommand(command, sizeof(command), response, responseSize, true);
    }
    catch (jtag_exception& e)
    {
        fprintf(stderr, "cannot read program counter: %s\n",
                e.what());
        throw;
    }

    unsigned long result = b4_to_u32(response + 1);
    delete [] response;

    // The JTAG box sees program memory as 16-bit wide locations. GDB
    // sees bytes. As such, double the PC value.
    result *= 2;

    cached_pc_is_valid = true;
    return cached_pc = result;
}

void jtag2::setProgramCounter(unsigned long pc)
{
    uchar *response;
    int responseSize;
    uchar command[5] = { CMND_WRITE_PC };

    u32_to_b4(command + 1, pc / 2);

    try
    {
        doJtagCommand(command, sizeof(command), response, responseSize);
    }
    catch (jtag_exception& e)
    {
        fprintf(stderr, "cannot write program counter: %s\n",
                e.what());
        throw;
    }

    delete [] response;

    cached_pc_is_valid = false;
}

PRAGMA_DIAG_PUSH
PRAGMA_DIAG_IGNORED("-Wunused-parameter")

void jtag2::resetProgram(bool possible_nSRST_ignored)
{
    if (proto == PROTO_DW) {
	/* The JTAG ICE mkII and Dragon do not respond correctly to
	 * the CMND_RESET command while in debugWire mode. */
	interruptProgram();
        setProgramCounter(0);
    } else {
	uchar cmd[2] = { CMND_RESET, 0x01 };
	uchar *resp;
	int respSize;

	doJtagCommand(cmd, 2, resp, respSize);
	delete [] resp;

	/* Await the BREAK event that is posted by the ICE. */
	bool bp, gdb;
	expectEvent(bp, gdb);
    }
}

PRAGMA_DIAG_POP

void jtag2::interruptProgram(void)
{
    uchar cmd[2] = { CMND_FORCED_STOP, 0x01 };
    uchar *resp;
    int respSize;

    doJtagCommand(cmd, 2, resp, respSize);
    delete [] resp;

    bool bp, gdb;
    expectEvent(bp, gdb);
}

void jtag2::resumeProgram(void)
{
    xmegaSendBPs();

    doSimpleJtagCommand(CMND_GO);

    cached_pc_is_valid = false;
}

void jtag2::expectEvent(bool &breakpoint, bool &gdbInterrupt)
{
    uchar *evtbuf;
    int evtSize;
    unsigned short seqno;

    evtSize = recvFrame(evtbuf, seqno);
    if (evtSize >= 0) {
	// XXX if not event, should push frame back into queue...
	// We really need a queue of received frames.
	if (seqno != 0xffff)
	    debugOut("Expected event packet, got other response");
	else if (!nonbreaking_events[evtbuf[8] - EVT_BREAK])
	{
	    switch (evtbuf[8])
	    {
		// Program stopped at some kind of breakpoint.
		case EVT_BREAK:
		    cached_pc = 2 * b4_to_u32(evtbuf + 9);
		    cached_pc_is_valid = true;
		    /* FALLTHROUGH */
		case EVT_EXT_RESET:
		case EVT_PDSB_BREAK:
		case EVT_PDSMB_BREAK:
		case EVT_PROGRAM_BREAK:
		    breakpoint = true;
		    break;

		case EVT_IDR_DIRTY:
		    // The program is still running at IDR dirty, so
		    // pretend a user break;
		    gdbInterrupt = true;
		    printf("\nIDR dirty: 0x%02x\n", evtbuf[9]);
		    break;

		    // Fatal debugWire errors, cannot continue
		case EVT_ERROR_PHY_FORCE_BREAK_TIMEOUT:
		case EVT_ERROR_PHY_MAX_BIT_LENGTH_DIFF:
		case EVT_ERROR_PHY_OPT_RECEIVE_TIMEOUT:
		case EVT_ERROR_PHY_OPT_RECEIVED_BREAK:
		case EVT_ERROR_PHY_RECEIVED_BREAK:
		case EVT_ERROR_PHY_RECEIVE_TIMEOUT:
		case EVT_ERROR_PHY_RELEASE_BREAK_TIMEOUT:
		case EVT_ERROR_PHY_SYNC_OUT_OF_RANGE:
		case EVT_ERROR_PHY_SYNC_TIMEOUT:
		case EVT_ERROR_PHY_SYNC_TIMEOUT_BAUD:
		case EVT_ERROR_PHY_SYNC_WAIT_TIMEOUT:
		    gdbInterrupt = true;
		    printf("\nFatal debugWIRE communication event: 0x%02x\n",
			   evtbuf[8]);
		    break;

		    // Other fatal errors, user could mask them off
		case EVT_ICE_POWER_ERROR_STATE:
		    gdbInterrupt = true;
		    printf("\nJTAG ICE mkII power failure\n");
		    break;

		case EVT_TARGET_POWER_OFF:
		    gdbInterrupt = true;
		    printf("\nTarget power turned off\n");
		    break;

		case EVT_TARGET_POWER_ON:
		    gdbInterrupt = true;
		    printf("\nTarget power returned\n");
		    break;

		case EVT_TARGET_SLEEP:
		    gdbInterrupt = true;
		    printf("\nTarget went to sleep\n");
		    break;

		case EVT_TARGET_WAKEUP:
		    gdbInterrupt = true;
		    printf("\nTarget went out of sleep\n");
		    break;

		    // Events where we want to continue
		case EVT_NONE:
		case EVT_RUN:
		    break;

		default:
		    gdbInterrupt = true;
		    printf("\nUnhandled JTAG ICE mkII event: 0x%0x2\n",
			   evtbuf[8]);
	    }
	}
	delete [] evtbuf;
    }
}

bool jtag2::eventLoop(void)
{
    int maxfd;
    fd_set readfds;
    bool breakpoint = false, gdbInterrupt = false;

    // Now that we are "going", wait for either a response from the JTAG
    // box or a nudge from GDB.

    for (;;)
      {
	  debugOut("Waiting for input.\n");

	  // Check for input from JTAG ICE (breakpoint, sleep, info, power)
	  // or gdb (user break)
	  FD_ZERO (&readfds);
	  if (gdbFileDescriptor != -1)
	    FD_SET (gdbFileDescriptor, &readfds);
	  FD_SET (jtagBox, &readfds);
	  if (gdbFileDescriptor != -1)
	    maxfd = jtagBox > gdbFileDescriptor ? jtagBox : gdbFileDescriptor;
	  else
	    maxfd = jtagBox;

	  int numfds = select(maxfd + 1, &readfds, 0, 0, 0);
	  if (numfds < 0)
              throw jtag_exception("GDB/JTAG ICE communications failure");

	  if (gdbFileDescriptor != -1 && FD_ISSET(gdbFileDescriptor, &readfds))
	    {
		int c = getDebugChar();
		if (c == 3) // interrupt
		  {
		      debugOut("interrupted by GDB\n");
		      gdbInterrupt = true;
		  }
		else
		    debugOut("Unexpected GDB input `%02x'\n", c);
	    }

	  if (FD_ISSET(jtagBox, &readfds))
	    {
		expectEvent(breakpoint, gdbInterrupt);
	    }

	  // We give priority to user interrupts
	  if (gdbInterrupt)
	      return false;
	  if (breakpoint)
	      return true;
      }
}


bool jtag2::jtagSingleStep(void)
{
    uchar cmd[3] = { CMND_SINGLE_STEP,
		     0x01, 0x01 };
    uchar *resp;
    int respSize, i = 2;
    bool result;

    xmegaSendBPs();

    cached_pc_is_valid = false;

    do
    {
        try
        {
            doJtagCommand(cmd, 3, resp, respSize);
        }
        catch (jtag_io_exception& e)
        {
            if (e.get_response() != RSP_ILLEGAL_MCU_STATE)
                throw;
            continue;
        }
	delete [] resp;
        break;
    }
    while (--i >= 0);
    if (i < 0)
        throw jtag_exception("Single-step failed");

    result = eventLoop();
    // this is weak as calling the loop may have consumed the break event we have to wait for
    if (!result)
    {
        bool bp, gdb;
        expectEvent(bp, gdb);
    }
    return result;

    // Stepping into a breakpoint on a 32 bit instruction in debugWire mode.
    //
    // If --disable-intr is given (interupts disabled before stepping):
    // * the ICE executes the original instruction
    // * the ICE removes the break instruction from flash
    //   (checked power cycling the target after the step)
    //
    // Otherwise, regardless of --ignore-intr, if the target breaks in the IVT:
    // * the ICE does NOT execute the original instruction
    // * the ICE does NOT remove the break instruction from flash
    // Calling jtag2::updateBreakpoints to delete the breakpoint
    // (to honour the 'z' packet after reporting a breakpoint stop from previous continute)
    // before stepping does not help.
    //
    // Note: If --ignore-intr is NOT geiven, GDB 12.1 handles breaking in the IVT
    // the same as jtag:handleInterrupt would,
    // creating a temporary breakpoint and continuing.
}

void jtag2::parseEvents(const char *evtlist)
{
    memset(nonbreaking_events, 0, sizeof nonbreaking_events);

    const struct
    {
        uchar num;
        const char *name;
    } evttable[] =
        {
            { EVT_BREAK,				"break" },
            { EVT_DEBUG,				"debug" },
            { EVT_ERROR_PHY_FORCE_BREAK_TIMEOUT,	"error_phy_force_break_timeout" },
            { EVT_ERROR_PHY_MAX_BIT_LENGTH_DIFF,	"error_phy_max_bit_length_diff" },
            { EVT_ERROR_PHY_OPT_RECEIVE_TIMEOUT,	"error_phy_opt_receive_timeout" },
            { EVT_ERROR_PHY_OPT_RECEIVED_BREAK,		"error_phy_opt_received_break" },
            { EVT_ERROR_PHY_RECEIVED_BREAK,		"error_phy_received_break" },
            { EVT_ERROR_PHY_RECEIVE_TIMEOUT,		"error_phy_receive_timeout" },
            { EVT_ERROR_PHY_RELEASE_BREAK_TIMEOUT,	"error_phy_release_break_timeout" },
            { EVT_ERROR_PHY_SYNC_OUT_OF_RANGE,		"error_phy_sync_out_of_range" },
            { EVT_ERROR_PHY_SYNC_TIMEOUT,		"error_phy_sync_timeout" },
            { EVT_ERROR_PHY_SYNC_TIMEOUT_BAUD,		"error_phy_sync_timeout_baud" },
            { EVT_ERROR_PHY_SYNC_WAIT_TIMEOUT,		"error_phy_sync_wait_timeout" },
            { EVT_RESULT_PHY_NO_ACTIVITY,		"result_phy_no_activity" },
            { EVT_EXT_RESET,				"ext_reset" },
            { EVT_ICE_POWER_ERROR_STATE,		"ice_power_error_state" },
            { EVT_ICE_POWER_OK,				"ice_power_ok" },
            { EVT_IDR_DIRTY,				"idr_dirty" },
            { EVT_NONE,					"none" },
            { EVT_PDSB_BREAK,				"pdsb_break" },
            { EVT_PDSMB_BREAK,				"pdsmb_break" },
            { EVT_PROGRAM_BREAK,			"program_break" },
            { EVT_RUN,					"run" },
            { EVT_TARGET_POWER_OFF,			"target_power_off" },
            { EVT_TARGET_POWER_ON,			"target_power_on" },
            { EVT_TARGET_SLEEP,				"target_sleep" },
            { EVT_TARGET_WAKEUP,			"target_wakeup" },
        };

    // parse the given comma-separated string
    const char *cp1, *cp2;
    cp1 = evtlist;
    while (*cp1 != '\0')
    {
        while (isspace(*cp1) || *cp1 == ',')
            cp1++;
        cp2 = cp1;
        while (*cp2 != '\0' && *cp2 != ',')
            cp2++;
        size_t l = cp2 - cp1;
        uchar evtval = 0;

        // Now, cp1 points to the name to parse, of length l
        for (unsigned int i = 0; i < sizeof evttable / sizeof evttable[0]; i++)
        {
            if (strncmp(evttable[i].name, cp1, l) == 0)
            {
                evtval = evttable[i].num;
                break;
            }
        }
        if (evtval == 0)
        {
            fprintf(stderr, "Warning: event name %.*s not matched\n",
                    (int)l, cp1);
        }
        else
        {
            nonbreaking_events[evtval - EVT_BREAK] = true;
        }

        cp1 = cp2;
    }
}

bool jtag2::jtagContinue(void)
{
    updateBreakpoints(); // download new bp configuration

    xmegaSendBPs();

    doSimpleJtagCommand(CMND_GO);

    return eventLoop();
}

void jtag2::setBreakOnChangeOfFlow(bool yesno)
{
    if (breakOnChangeOfFlow != yesno) {
        uchar command[1] = { CMND_SET_PARAMETER };

        this->setJtagParameter(PAR_BREAK_ON_CHANGE_FLOW, command, yesno);

        breakOnChangeOfFlow = yesno;
    }
}

bool jtag2::jtagRunToAddress(unsigned long toPC)
{
    uchar *response;
    int responseSize;
    uchar command[5] = { CMND_RUN_TO_ADDR };

    u32_to_b4(command + 1, toPC / 2);

    try
    {
        doJtagCommand(command, sizeof(command), response, responseSize, true);
    }
    catch (jtag_exception& e)
    {
        fprintf(stderr, "cannot run to address: %s\n",
                e.what());
        throw;
    }

    delete [] response;

    cached_pc_is_valid = false;

    bool result = eventLoop();
    // this is weak as calling the loop may have consumed the break event we have to wait for
    if (!result)
    {
        bool bp, gdb;
        expectEvent(bp, gdb);
    }
    return result;
}

bool jtag2::handleInterrupt(void)
{
    bool result;

    debugOut("INTERRUPT\n");
    unsigned int intrSP = readSP();
    unsigned int retPC = readBWord(intrSP + 1) << 1;
    debugOut("INT SP = %x, retPC = %x\n", intrSP, retPC);

    if (proto != PROTO_DW)
        setBreakOnChangeOfFlow(false);

    for (;;)
    {
        result = jtagRunToAddress(retPC);

        if (!result) // user interrupt
            break;

        // We check that SP is > intrSP. If SP <= intrSP, this is just
        // an unrelated excursion to retPC
        if (readSP() > intrSP)
            break;
    }
    return result;
}

bool jtag2::rangeStep(unsigned long start, unsigned long end)
{
    if (proto == PROTO_DW)
        return jtag::rangeStep(start, end);

    // Disable interrupts while stepping, like in AVR Studio.
    // Compared to using only the --ignore-intr option,
    // this improves debugging speed because the target will not
    // break at every interrupt while running to the end address.
    // However, as explained below, the inferior program can enable interrupts
    // in a GDB next or step command,
    // for example executing a SEI or RETI instruction.
    // In this case, if an interrupt occurs, the target breaks
    // in the vectors table unless --ignore-intr is given.
    // This is why --ignore-intr is recommended in conjunction with --disable-intr.
    //
    // A GDB step or next command may require multiple interactions
    // with AVaRICE.
    // Initially GDB asks AVaRICE to continue to the end address.
    // However this rarely happens due to normal changes in the program flow,
    // like in the case of a branch or a call to subroutine.
    // When this happens the target breaks.
    // We re-enable interrupts and return control to GDB
    // which needs to figure out what to do next.
    // This description is not exhaustive but if GDB realizes that end address
    // is reachable, it creates a breakpoint and asks Avarice to continue.
    // In this phase interrupts are enabled and they can be serviced.
    //
    // Before returning from this function we have to decide on interrupts.
    // This is not trivial.
    // SEI, CLI and RETI instructions modify the global interrupt flag.
    // There is no easy way to know what intructions the program has executed
    // so we cannot simply restore interrupts.
    // If the interrupt flag was clear then
    // it is either still clear or it has been set by the program.
    // In this case we don't need to do anything.
    // If the interrupt flag was set and the program has executed
    // an unbalanced CLI (not followed by a SEI/RETI), re-enabling
    // interrupts is the wrong decision.
    // Since the whole point is to debug code where interrupts are enabled,
    // the above decision of re-enabling interrupts is often the right one
    // but if we happen to step over a function which disable interrupts,
    // we have to manually clear the global interrupt flag after the step.

    bool result;
    bool interruptsEnabled;

    unsigned long pc = getProgramCounter();
    while (doubleWordSwBreakpointAt(pc))
    {
        result = singleStep();
        if (!result)
            break;

        pc = getProgramCounter();
        if (!(pc >= start && pc < end))
            break;
    }

    if (!result)
        return result;

    if (disableInterrupts)
    {
        unsigned char sreg = readSREGAndWriteIntrEnable(false);
        interruptsEnabled = sreg & 0x80;
    }

    setBreakOnChangeOfFlow(true);

    do
    {
        result = jtagRunToAddress(end);
        if (!result)
            break;

        pc = getProgramCounter();
        if (codeBreakpointAt(pc))
            break;

        // assume interrupt when PC goes into interrupt table
        if (ignoreInterrupts && pc < deviceDef->vectors_end)
        {
            result = handleInterrupt();
            if (!result)
                break;

            pc = getProgramCounter();
        }
    } while (pc >= start && pc < end);

    if (disableInterrupts && interruptsEnabled)
        readSREGAndWriteIntrEnable(true);

    return result;
}
