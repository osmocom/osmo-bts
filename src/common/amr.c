#include <stdint.h>
#include <errno.h>

#include <osmocom/core/logging.h>

#include <osmo-bts/logging.h>
#include <osmo-bts/amr.h>

/* Reasonable defaults for AMR-FR and AMR-HR rate configuration.
 * The values are taken from 3GPP TS 51.010-1 (version 13.11.0).
 * See 14.2.19.4.1 and 14.2.20.4.1 for AMR-FR and AMR-HR, respectively.
 *
 *                   ^ C/I (dB)             |  FR  /  HR  |
 *            |      |
 *            |      |
 *   MODE4    |      |
 *          = |  ----+----  THR_MX_Up(3)    | 20.5 / 18.0 |
 *          | |      |
 *          | =  ----+----  THR_MX_Dn(4)    | 18.5 / 16.0 |
 *   MODE3  |        |
 *          | =  ----+----  THR_MX_Up(2)    | 14.5 / 14.0 |
 *          | |      |
 *          = |  ----+----  THR_MX_Dn(3)    | 12.5 / 12.0 |
 *   MODE2    |      |
 *          = |  ----+----  THR_MX_Up(1)    |  8.5 / 10.0 |
 *          | |      |
 *          | =  ----+----  THR_MX_Dn(2)    |  6.5 /  8.0 |
 *   MODE1  |        |
 *          |        |
 *          |        |
 */
static const struct gsm48_multi_rate_conf amr_fr_mr_cfg_def = {
	.m4_75 = 1,
	.m5_90 = 1,
	.m7_95 = 1,
	.m12_2 = 1,
};
static const struct amr_mode amr_fr_bts_mode_def[] = {
	{
		.mode = 0, /* 4.75k */
		.threshold = 13, /* THR_MX_Dn(2): 6.5 dB */
		.hysteresis = 4, /* THR_MX_Up(1): 8.5 dB */
	},
	{
		.mode = 2, /* 5.90k */
		.threshold = 25, /* THR_MX_Dn(3): 12.5 dB */
		.hysteresis = 4, /* THR_MX_Up(2): 14.5 dB */
	},
	{
		.mode = 5, /* 7.95k */
		.threshold = 37, /* THR_MX_Dn(4): 18.5 dB */
		.hysteresis = 4, /* THR_MX_Up(3): 20.5 dB */
	},
	{
		.mode = 7, /* 12.2k */
		/* this is the last mode, so no threshold */
	},
};

static const struct gsm48_multi_rate_conf amr_hr_mr_cfg_def = {
	.m4_75 = 1,
	.m5_90 = 1,
	.m6_70 = 1,
	.m7_95 = 1,
};
static const struct amr_mode amr_hr_bts_mode_def[] = {
	{
		.mode = 0, /* 4.75k */
		.threshold = 16, /* THR_MX_Dn(2):  8.0 dB */
		.hysteresis = 4, /* THR_MX_Up(1): 10.0 dB */
	},
	{
		.mode = 2, /* 5.90k */
		.threshold = 24, /* THR_MX_Dn(3): 12.0 dB */
		.hysteresis = 4, /* THR_MX_Up(2): 14.0 dB */
	},
	{
		.mode = 3, /* 6.70k */
		.threshold = 32, /* THR_MX_Dn(4): 16.0 dB */
		.hysteresis = 4, /* THR_MX_Up(3): 18.0 dB */
	},
	{
		.mode = 5, /* 7.95k */
		/* this is the last mode, so no threshold */
	},
};

void amr_log_mr_conf(int ss, int logl, const char *pfx,
		     struct amr_multirate_conf *amr_mrc)
{
	int i;

	LOGP(ss, logl, "%s AMR MR Conf: num_modes=%u",
		pfx, amr_mrc->num_modes);

	for (i = 0; i < amr_mrc->num_modes; i++)
		LOGPC(ss, logl, ", mode[%u] = %u/%u/%u",
			i, amr_mrc->bts_mode[i].mode,
			amr_mrc->bts_mode[i].threshold,
			amr_mrc->bts_mode[i].hysteresis);
	LOGPC(ss, logl, "\n");
}

static inline int get_amr_mode_idx(const struct amr_multirate_conf *amr_mrc,
				   uint8_t cmi)
{
	unsigned int i;
	for (i = 0; i < amr_mrc->num_modes; i++) {
		if (amr_mrc->bts_mode[i].mode == cmi)
			return i;
	}
	return -EINVAL;
}

static inline uint8_t set_cmr_mode_idx(const struct amr_multirate_conf *amr_mrc,
				       uint8_t cmr)
{
	int rc;

	/* Codec Mode Request is in upper 4 bits of RTP payload header,
	 * and we simply copy the CMR into the CMC */
	if (cmr == 0xF) {
		/* FIXME: we need some state about the last codec mode */
		return 0;
	}

	rc = get_amr_mode_idx(amr_mrc, cmr);
	if (rc < 0) {
		/* FIXME: we need some state about the last codec mode */
		LOGP(DRTP, LOGL_INFO, "RTP->L1: overriding CMR %u\n", cmr);
		return 0;
	}
	return rc;
}

static inline uint8_t set_cmi_mode_idx(const struct amr_multirate_conf *amr_mrc,
				       uint8_t cmi)
{
	int rc = get_amr_mode_idx(amr_mrc, cmi);
	if (rc < 0) {
		LOGP(DRTP, LOGL_ERROR, "AMR CMI %u not part of AMR MR set\n",
		     cmi);
		return 0;
	}
	return rc;
}

void amr_set_mode_pref(uint8_t *data, const struct amr_multirate_conf *amr_mrc,
		      uint8_t cmi, uint8_t cmr)
{
	data[0] = set_cmi_mode_idx(amr_mrc, cmi);
	data[1] = set_cmr_mode_idx(amr_mrc, cmr);
}

/* parse a GSM 04.08 MultiRate Config IE (10.5.2.21aa) in a more
 * comfortable internal data structure */
int amr_parse_mr_conf(struct amr_multirate_conf *amr_mrc,
		      const uint8_t *mr_conf, unsigned int len)
{
	uint8_t num_codecs = 0;
	int i, j = 0;

	if (len < 2) {
		LOGP(DRSL, LOGL_ERROR, "AMR Multirate IE is too short (%u)\n", len);
		goto ret_einval;
	}

	if ((mr_conf[0] >> 5) != 1) {
		LOGP(DRSL, LOGL_ERROR, "AMR Multirate Version %u unknown\n", (mr_conf[0] >> 5));
		goto ret_einval;
	}

	/* check number of active codecs */
	for (i = 0; i < 8; i++) {
		if (mr_conf[1] & (1 << i))
			num_codecs++;
	}

	/* check for minimum length */
	if (num_codecs == 0 ||
	    (num_codecs == 1 && len < 2) ||
	    (num_codecs == 2 && len < 4) ||
	    (num_codecs == 3 && len < 5) ||
	    (num_codecs == 4 && len < 6) ||
	    (num_codecs > 4)) {
		LOGP(DRSL, LOGL_ERROR, "AMR Multirate with %u modes len=%u "
		     "not possible\n", num_codecs, len);
		goto ret_einval;
	}

	/* copy the first two octets of the IE */
	amr_mrc->gsm48_ie[0] = mr_conf[0];
	amr_mrc->gsm48_ie[1] = mr_conf[1];

	amr_mrc->num_modes = num_codecs;

	for (i = 0; i < 8; i++) {
		if (mr_conf[1] & (1 << i)) {
			amr_mrc->bts_mode[j++].mode = i;
		}
	}

	/* skip the first two octets of the IE */
	mr_conf += 2;

	if (num_codecs >= 2) {
		amr_mrc->bts_mode[0].threshold = mr_conf[0] & 0x3F;
		amr_mrc->bts_mode[0].hysteresis = mr_conf[1] >> 4;
	}
	if (num_codecs >= 3) {
		amr_mrc->bts_mode[1].threshold =
			((mr_conf[1] & 0xF) << 2) | (mr_conf[2] >> 6);
		amr_mrc->bts_mode[1].hysteresis = (mr_conf[2] >> 2) & 0xF;
	}
	if (num_codecs >= 4) {
		amr_mrc->bts_mode[2].threshold =
			((mr_conf[2] & 0x3) << 4) | (mr_conf[3] >> 4);
		amr_mrc->bts_mode[2].hysteresis = mr_conf[3] & 0xF;
	}

	return num_codecs;

ret_einval:
	return -EINVAL;
}


/*! \brief determine AMR initial codec mode for given logical channel 
 *  \returns integer between 0..3 for AMR codce mode 1..4 */
unsigned int amr_get_initial_mode(struct gsm_lchan *lchan)
{
	struct amr_multirate_conf *amr_mrc = &lchan->tch.amr_mr;
	struct gsm48_multi_rate_conf *mr_conf =
			(struct gsm48_multi_rate_conf *) amr_mrc->gsm48_ie;

	if (mr_conf->icmi) {
		/* initial mode given, coding in TS 05.09 3.4.1 */
		return mr_conf->smod;
	} else {
		/* implicit rule according to TS 05.09 Chapter 3.4.3 */
		switch (amr_mrc->num_modes) {
		case 2:
		case 3:
			/* return the most robust */
			return 0;
		case 4:
			/* return the second-most robust */
			return 1;
		case 1:
		default:
			/* return the only mode we have */
			return 0;
		}
	}
}

void amr_init_mr_conf_def(struct gsm_lchan *lchan)
{
	const struct gsm48_multi_rate_conf *mr_cfg;
	const struct amr_mode *bts_mode;
	unsigned int num_modes;

	if (lchan->type == GSM_LCHAN_TCH_F) {
		num_modes = ARRAY_SIZE(amr_fr_bts_mode_def);
		bts_mode = &amr_fr_bts_mode_def[0];
		mr_cfg = &amr_fr_mr_cfg_def;
	} else {
		num_modes = ARRAY_SIZE(amr_hr_bts_mode_def);
		bts_mode = &amr_hr_bts_mode_def[0];
		mr_cfg = &amr_hr_mr_cfg_def;
	}

	memcpy(lchan->tch.amr_mr.gsm48_ie, mr_cfg,
	       sizeof(lchan->tch.amr_mr.gsm48_ie));
	memcpy(&lchan->tch.amr_mr.bts_mode[0], &bts_mode[0],
	       sizeof(lchan->tch.amr_mr.bts_mode));
	lchan->tch.amr_mr.num_modes = num_modes;
}
