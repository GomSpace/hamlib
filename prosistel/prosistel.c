/*
 *  Hamlib Rotator backend - GS-232A
 *  Copyright (c) 2001-2012 by Stephane Fillod
 *
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <math.h>

#include "hamlib/rotator.h"
#include "serial.h"
#include "misc.h"
#include "register.h"

#define EOM "\r"
#define REPLY_EOM "\r\n"

#define BUFSZ 64

/**
 * gs232a_transaction
 *
 * cmdstr - Command to be sent to the rig.
 * data - Buffer for reply string.  Can be NULL, indicating that no reply is
 *        is needed, but answer will still be read.
 * data_len - in: Size of buffer. It is the caller's responsibily to provide
 *            a large enough buffer for all possible replies for a command.
 *
 * returns:
 *   RIG_OK  -  if no error occured.
 *   RIG_EIO  -  if an I/O error occured while sending/receiving data.
 *   RIG_ETIMEOUT  -  if timeout expires without any characters received.
 *   RIG_REJECTED  -  if a negative acknowledge was received or command not
 *                    recognized by rig.
 */
static int
prosistel_transaction (ROT *rot, const char *cmdstr,
				char *data, size_t data_len)
{
    struct rot_state *rs;
    int retval;
    int retry_read = 0;
    char replybuf[BUFSZ];

    rs = &rot->state;

transaction_write:

    serial_flush(&rs->rotport);

    if (cmdstr) {
        retval = write_block(&rs->rotport, cmdstr, strlen(cmdstr));
        if (retval != RIG_OK)
            goto transaction_quit;
    }

    /* Always read the reply to know whether the cmd went OK */
    if (!data)
        data = replybuf;
    if (!data_len)
        data_len = BUFSZ;

    memset(data,0,data_len);
    retval = read_string(&rs->rotport, data, data_len, REPLY_EOM, strlen(REPLY_EOM));
    if (retval < 0) {
        if (retry_read++ < rot->state.rotport.retry)
            goto transaction_write;
        goto transaction_quit;
    }

#if 0
    /* Check that command termination is correct */
    if (strchr(REPLY_EOM, data[strlen(data)-1])==NULL) {
        rig_debug(RIG_DEBUG_ERR, "%s: Command is not correctly terminated '%s'\n", __FUNCTION__, data);
        if (retry_read++ < rig->state.rotport.retry)
            goto transaction_write;
        retval = -RIG_EPROTO;
        goto transaction_quit;
    }
#endif

    if (data[0] == '?') {
	    /* Invalid command */
	    rig_debug(RIG_DEBUG_VERBOSE, "%s: Error for '%s': '%s'\n",
			    __FUNCTION__, cmdstr, data);
	    retval = -RIG_EPROTO;
	    goto transaction_quit;
    }

    retval = RIG_OK;
transaction_quit:
    return retval;
}


static int
prosistel_rot_set_position(ROT *rot, azimuth_t az, elevation_t el)
{
    char cmdstr[64];
    int retval;
    unsigned u_az, u_el;

    rig_debug(RIG_DEBUG_TRACE, "%s called: %f %f\n", __FUNCTION__, az, el);

    u_az = (unsigned)rint(az * 10);
    u_el = (unsigned)rint(el * 10);

    sprintf(cmdstr, "\x02""AG%04u\r", u_az);
    retval = prosistel_transaction(rot, cmdstr, NULL, 0);
    if (retval != RIG_OK)
        return retval;

    sprintf(cmdstr, "\x02""BG%04u\r", u_el);
    retval = prosistel_transaction(rot, cmdstr, NULL, 0);
    if (retval != RIG_OK)
        return retval;

    return RIG_OK;
}

static int
prosistel_rot_get_position(ROT *rot, azimuth_t *az, elevation_t *el)
{
    char posbuf[32];
    int retval, angle;

    rig_debug(RIG_DEBUG_TRACE, "%s called\n", __FUNCTION__);

	/* Get azimuth */
	char getpos_a_command[] = "\x02""A?\r";
    retval = prosistel_transaction(rot, getpos_a_command, posbuf, sizeof(posbuf));
    if (retval != RIG_OK || strlen(posbuf) < 10) {
        return retval < 0 ? retval : -RIG_EPROTO;
    }

    if (sscanf(posbuf+5, "%d", &angle) != 1) {
        rig_debug(RIG_DEBUG_ERR, "%s: wrong reply '%s'\n", __FUNCTION__, posbuf);
        return -RIG_EPROTO;
    }
    *az = (azimuth_t)angle / 10;

    /* Get elevation */
    char getpos_b_command[] = "\x02""B?\r";
	retval = prosistel_transaction(rot, getpos_b_command, posbuf, sizeof(posbuf));
	if (retval != RIG_OK || strlen(posbuf) < 10) {
		return retval < 0 ? retval : -RIG_EPROTO;
	}

	if (sscanf(posbuf+7, "%d", &angle) != 1) {
        rig_debug(RIG_DEBUG_ERR, "%s: wrong reply '%s'\n", __FUNCTION__, posbuf);
        return -RIG_EPROTO;
    }
    *el = (elevation_t)angle / 10;

    rig_debug(RIG_DEBUG_TRACE, "%s: (az, el) = (%.1f, %.1f)\n",
		   __FUNCTION__, *az, *el);

    return RIG_OK;
}

static int
prosistel_rot_stop(ROT *rot)
{
    rig_debug(RIG_DEBUG_TRACE, "%s called\n", __FUNCTION__);

    /* All Stop */
    prosistel_transaction(rot, "\x02""AG9999\r", NULL, 0);
    prosistel_transaction(rot, "\x02""BG9999\r", NULL, 0);

    return RIG_OK;
}

static int
prosistel_rot_open(ROT *rot)
{
    int retval;

	rig_debug(RIG_DEBUG_TRACE, "%s called\n", __FUNCTION__);

	char databuf[1000];

    prosistel_transaction(rot, "\x02""AL\r", databuf, 1000);
    usleep(100000);
    prosistel_transaction(rot, "\x02""BL\r", databuf, 1000);

    /* Send STOP */
#if 1
    retval = prosistel_transaction(rot, "\x02""AS\r", databuf, 1000);
	if (retval != RIG_OK) {
		return retval < 0 ? retval : -RIG_EPROTO;
	}
	retval = prosistel_transaction(rot, "\x02""BS\r", databuf, 1000);
	if (retval != RIG_OK) {
		return retval < 0 ? retval : -RIG_EPROTO;
	}
#endif

	/* Update EEPROM parameters */
	//char type = 'L';
	//char lowlim =

    return RIG_OK;
}


/* ************************************************************************* */
/*
 * Generic rotator capabilities.
 */

const struct rot_caps prosistel_rot_caps = {
  .rot_model =      ROT_MODEL_PROSISTEL,
  .model_name =     "Combo Desk Top",
  .mfg_name =       "Pro.Sis.Tel",
  .version =        "0.1",
  .copyright = 	    "LGPL",
  .status =         RIG_STATUS_BETA,
  .rot_type =       ROT_TYPE_AZEL,
  .port_type =      RIG_PORT_SERIAL,
  .serial_rate_min =   9600,
  .serial_rate_max =   9600,
  .serial_data_bits =  8,
  .serial_stop_bits =  1,
  .serial_parity =  RIG_PARITY_NONE,
  .serial_handshake =  RIG_HANDSHAKE_NONE,
  .write_delay =  0,
  .post_write_delay =  0,
  .timeout =  400,
  .retry =  3,

  .min_az = 	0.0,
  .max_az =  	450.0,
  .min_el = 	0.0,
  .max_el =  	90.0,

  .get_position =  prosistel_rot_get_position,
  .set_position =  prosistel_rot_set_position,
  .stop = 	       prosistel_rot_stop,
  .rot_open =      prosistel_rot_open,
};


/* ************************************************************************* */

DECLARE_INITROT_BACKEND(prosistel)
{
	rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __FUNCTION__);

    rot_register(&prosistel_rot_caps);

	return RIG_OK;
}

/* ************************************************************************* */
/* end of file */

