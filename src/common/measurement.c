
#include <stdint.h>
#include <errno.h>

#include <osmocom/gsm/gsm_utils.h>

#include <osmo-bts/gsm_data.h>
#include <osmo-bts/logging.h>
#include <osmo-bts/measurement.h>

/* TS 05.08, Chapter 8.4.1 */
/* measurement period ends at fn % 104 == ? */
static const uint8_t tchf_meas_rep_fn104[] = {
	[0] =	103,
	[1] =	12,
	[2] =	25,
	[3] =	38,
	[4] =	51,
	[5] =	64,
	[6] =	77,
	[7] =	90,
};
static const uint8_t tchh0_meas_rep_fn104[] = {
	[0] =	103,
	[1] =	103,
	[2] =	25,
	[3] =	25,
	[4] =	51,
	[5] =	51,
	[6] =	77,
	[7] =	77,
};
static const uint8_t tchh1_meas_rep_fn104[] = {
	[0] =	12,
	[1] =	12,
	[2] =	38,
	[3] =	38,
	[4] =	64,
	[5] =	64,
	[6] =	90,
	[7] =	90,
};

/* determine if a measurement period ends at the given frame number */
static int is_meas_complete(enum gsm_phys_chan_config pchan, unsigned int ts,
			    unsigned int subch, uint32_t fn)
{
	unsigned int fn_mod;
	const uint8_t *tbl;
	int rc = 0;

	if (ts >= 8)
		return -EINVAL;
	if (pchan >= _GSM_PCHAN_MAX)
		return -EINVAL;

	switch (pchan) {
	case GSM_PCHAN_TCH_F:
		fn_mod = fn % 104;
		if (tchf_meas_rep_fn104[ts] == fn_mod)
			rc = 1;
		break;
	case GSM_PCHAN_TCH_H:
		fn_mod = fn % 104;
		if (subch == 0)	
			tbl = tchh0_meas_rep_fn104;
		else
			tbl = tchh1_meas_rep_fn104;
		if (tbl[ts] == fn_mod)
			rc = 1;
		break;
	case GSM_PCHAN_SDCCH8_SACCH8C:
		fn_mod = fn % 102;
		if (fn_mod == 11)
			rc = 1;
		break;
	case GSM_PCHAN_CCCH_SDCCH4:
		fn_mod = fn % 102;
		if (fn_mod == 36)
			rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

/* receive a L1 uplink measurement from L1 */
int lchan_new_ul_meas(struct gsm_lchan *lchan, struct bts_ul_meas *ulm)
{
	DEBUGP(DMEAS, "%s adding measurement, num_ul_meas=%d\n",
		gsm_lchan_name(lchan), lchan->meas.num_ul_meas);

	if (lchan->meas.num_ul_meas >= ARRAY_SIZE(lchan->meas.uplink)) {
		LOGP(DMEAS, LOGL_NOTICE, "%s no space for uplink measurement\n",
			gsm_lchan_name(lchan));
		return -ENOSPC;
	}

	memcpy(&lchan->meas.uplink[lchan->meas.num_ul_meas++], ulm,
		sizeof(*ulm));

	return 0;
}

/* input: BER in steps of .01%, i.e. percent/100 */
static uint8_t ber10k_to_rxqual(uint32_t ber10k)
{
	/* 05.08 / 8.2.4 */
	if (ber10k < 20)
		return 0;
	if (ber10k < 40)
		return 1;
	if (ber10k < 80)
		return 2;
	if (ber10k < 160)
		return 3;
	if (ber10k < 320)
		return 4;
	if (ber10k < 640)
		return 5;
	if (ber10k < 1280)
		return 6;
	return 7;
}

int lchan_meas_check_compute(struct gsm_lchan *lchan, uint32_t fn)
{
	uint32_t ber_full_sum = 0;
	uint32_t irssi_full_sum = 0;
	uint32_t ber_sub_sum = 0;
	uint32_t irssi_sub_sum = 0;
	int32_t taqb_sum = 0;
	unsigned int num_meas_sub = 0;
	int i;

	/* if measurement period is not complete, abort */
	if (!is_meas_complete(lchan->ts->pchan, lchan->ts->nr,
			      lchan->nr, fn))
		return 0;

	/* if there are no measurements, skip computation */
	if (lchan->meas.num_ul_meas == 0)
		return 0;

	/* compute the actual measurements */

	/* step 1: add up */
	for (i = 0; i < lchan->meas.num_ul_meas; i++) {
		struct bts_ul_meas *m = &lchan->meas.uplink[i];

		ber_full_sum += m->ber10k;
		irssi_full_sum += m->inv_rssi;
		taqb_sum += m->ta_offs_qbits;

		if (m->is_sub) {
			num_meas_sub++;
			ber_sub_sum += m->ber10k;
			irssi_sub_sum += m->inv_rssi;
		}
	}

	/* step 2: divide */
	ber_full_sum = ber_full_sum / lchan->meas.num_ul_meas;
	irssi_full_sum = irssi_full_sum / lchan->meas.num_ul_meas;
	taqb_sum = taqb_sum / lchan->meas.num_ul_meas;

	if (num_meas_sub) {
		ber_sub_sum = ber_sub_sum / num_meas_sub;
		irssi_sub_sum = irssi_sub_sum / num_meas_sub;
	}

	DEBUGP(DMEAS, "%s Computed TA(% 4dqb) BER-FULL(%2u.%02u%%), RSSI-FULL(-%3udBm), "
		"BER-SUB(%2u.%02u%%), RSSI-SUB(-%3udBm)\n", gsm_lchan_name(lchan),
		taqb_sum, ber_full_sum/100,
		ber_full_sum%100, irssi_full_sum, ber_sub_sum/100, ber_sub_sum%100,
		irssi_sub_sum);

	/* store results */
	lchan->meas.res.rxlev_full = dbm2rxlev((int)irssi_full_sum * -1);
	lchan->meas.res.rxlev_sub = dbm2rxlev((int)irssi_sub_sum * -1);
	lchan->meas.res.rxqual_full = ber10k_to_rxqual(ber_full_sum);
	lchan->meas.res.rxqual_sub = ber10k_to_rxqual(ber_sub_sum);

	lchan->meas.flags |= LC_UL_M_F_RES_VALID;
	lchan->meas.num_ul_meas = 0;

	/* send a signal indicating computation is complete */

	return 1;
}

/* build the 3 byte RSL uplinke measurement IE content */
int lchan_build_rsl_ul_meas(struct gsm_lchan *lchan, uint8_t *buf)
{
	buf[0] = (lchan->meas.res.rxlev_full & 0x3f); /* FIXME: DTXu support */
	buf[1] = (lchan->meas.res.rxlev_sub & 0x3f);
	buf[2] = ((lchan->meas.res.rxqual_full & 7) << 3) |
					(lchan->meas.res.rxqual_sub & 7);

	return 3;
}

int ts_meas_check_compute(struct gsm_bts_trx_ts *ts, uint32_t fn)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ts->lchan); i++) {
		struct gsm_lchan *lchan = &ts->lchan[i];

		if (lchan->state != LCHAN_S_ACTIVE)
			continue;

		switch (lchan->type) {
		case GSM_LCHAN_SDCCH:
		case GSM_LCHAN_TCH_F:
		case GSM_LCHAN_TCH_H:
		case GSM_LCHAN_PDTCH:
			lchan_meas_check_compute(lchan, fn);
			break;
		default:
			break;
		}
	}
	return 0;
}

/* needs to be called once every TDMA frame ! */
int trx_meas_check_compute(struct gsm_bts_trx *trx, uint32_t fn)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(trx->ts); i++) {
		struct gsm_bts_trx_ts *ts = &trx->ts[i];
		ts_meas_check_compute(ts, fn);
	}
	return 0;
}
