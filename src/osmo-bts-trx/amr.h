#ifndef _TRX_AMR_H
#define _TRX_AMR_H

int amr_decompose_payload(uint8_t *payload, int payload_len, uint8_t *_cmr,
	uint8_t *_ft, uint8_t *_bfi);
int amr_compose_payload(uint8_t *payload, uint8_t cmr, uint8_t ft, uint8_t bfi);

#endif /* _TRX_AMR_H */
