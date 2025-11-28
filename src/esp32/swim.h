#pragma once

#include <stdint.h>
#include <stddef.h>

#define SWIM_ERROR_READ_BIT_TIMEOUT -1
#define SWIM_ERROR_INVALID_TARGET_ID -2
#define SWIM_ERROR_PARITY -3
#define SWIM_ERROR_NACK -4
#define SWIM_ERROR_SYNC_TIMEOUT_1 -5
#define SWIM_ERROR_SYNC_TIMEOUT_2 -6

void generate_len_and_address_spec(uint8_t *dest, size_t len, uint32_t address);
int rotf(const uint8_t *len_and_address_spec, uint8_t *dest);
int wotf(const uint8_t *data);
int srst(void);
int swim_entry(void);
void reset(int on);
void swim_init(void);
