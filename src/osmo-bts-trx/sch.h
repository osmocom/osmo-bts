#ifndef _SCH_H
#define _SCH_H

int sch_decode(uint8_t *sb_info, sbit_t *burst);
int sch_encode(ubit_t *burst, uint8_t *sb_info);

#endif /* _SCH_H */
