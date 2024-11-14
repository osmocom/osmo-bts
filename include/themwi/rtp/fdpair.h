/*
 * This header file provides API declaration for twrtp_bind_fdpair()
 * standalone function.
 */

#pragma once

#include <stdint.h>

int twrtp_bind_fdpair(const char *ip, uint16_t port, int *fd_rtp, int *fd_rtcp);
