/*
 *  Hamlib RFT backend - EKD-500 description
 *  Copyright (c) 2003 by Thomas B. Ruecker
 *
 *	$Id: ekd500.c,v 1.1 2003-10-07 22:15:49 fillods Exp $
 *
 *   This library is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU Library General Public License as
 *   published by the Free Software Foundation; either version 2 of
 *   the License, or (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Library General Public License for more details.
 *
 *   You should have received a copy of the GNU Library General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>

#include <hamlib/rig.h>
#include "rft.h"


#define EKD500_MODES (RIG_MODE_SSB|RIG_MODE_CW|RIG_MODE_RTTY|RIG_MODE_AM|RIG_MODE_FM)

#define EKD500_FUNC (RIG_FUNC_NONE)

#define EKD500_LEVEL_ALL (RIG_LEVEL_NONE)

#define EKD500_PARM_ALL (RIG_PARM_NONE)

#define EKD500_VFO (RIG_VFO_A)

#define EKD500_VFO_OPS (RIG_OP_NONE)


/*
 * EKD-500 rig capabilities.
 *
 * Documentation:
 *   http://kahuna.sdsu.edu/~mechtron/PremRxPage/ekd500.htm
 */

const struct rig_caps ekd500_caps = {
.rig_model =  RIG_MODEL_EKD500,
.model_name = "EKD-500",
.mfg_name =  "RFT",
.version =  "0.1",
.copyright =  "LGPL",
.status =  RIG_STATUS_NEW,
.rig_type =  RIG_TYPE_RECEIVER,
.ptt_type =  RIG_PTT_NONE,
.dcd_type =  RIG_DCD_NONE,
.port_type =  RIG_PORT_SERIAL,
.serial_rate_min =  300,
.serial_rate_max =  2400,
.serial_data_bits =  7,
.serial_stop_bits =  1,
.serial_parity =  RIG_PARITY_EVEN,
.serial_handshake =  RIG_HANDSHAKE_HARDWARE,
.write_delay =  0,
.post_write_delay =  1,
.timeout =  200,
.retry =  3,

.has_get_func =  EKD500_FUNC,
.has_set_func =  EKD500_FUNC,
.has_get_level =  EKD500_LEVEL_ALL,
.has_set_level =  RIG_LEVEL_SET(EKD500_LEVEL_ALL),
.has_get_parm =  EKD500_PARM_ALL,
.has_set_parm =  RIG_PARM_SET(EKD500_PARM_ALL),
.level_gran =  {},
.parm_gran =  {},
.ctcss_list =  NULL,
.dcs_list =  NULL,
.preamp =   { RIG_DBLST_END },
.attenuator =   { RIG_DBLST_END },
.max_rit =  Hz(0),
.max_xit =  Hz(0),
.max_ifshift =  Hz(0),
.targetable_vfo =  0,
.transceive =  RIG_TRN_OFF,
.bank_qty =   0,
.chan_desc_sz =  7,
.vfo_ops =  EKD500_VFO_OPS,

.chan_list =  {
		RIG_CHAN_END,	/* FIXME */
	},

.rx_range_list1 =  { 
	{kHz(10),MHz(30),EKD500_MODES,-1,-1,EKD500_VFO},
	RIG_FRNG_END,
  },
.tx_range_list1 =  { RIG_FRNG_END, },
.rx_range_list2 =  {
	{kHz(10),MHz(30),EKD500_MODES,-1,-1,EKD500_VFO},
	RIG_FRNG_END,
  },
.tx_range_list2 =  { RIG_FRNG_END, },

.tuning_steps =  {
	 {EKD500_MODES,10},
	 RIG_TS_END,
	},
        /* mode/filter list, remember: order matters! */
.filters =  {
		{RIG_MODE_SSB|RIG_MODE_CW|RIG_MODE_RTTY|RIG_MODE_AM, kHz(2.2)},
		{RIG_MODE_FM, kHz(12)},
		RIG_FLT_END,
	},
.priv =  NULL,

.set_freq =  rft_set_freq,
};

/*
 * Function definitions below
 */
