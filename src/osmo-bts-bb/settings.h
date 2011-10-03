#ifndef _SETTINGS_H
#define _SETTINGS_H

int set_show_layout(int i);
int set_show_layouts(void);
int set_layout_by_name(const char *name);
int set_num_phones(int i);
uint8_t set_get_tx_mask(int i, int p);
uint8_t set_get_rx_mask(int i, int p);

#endif /* _SETTINGS_H */
