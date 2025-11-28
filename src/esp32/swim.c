/**
 * ESP32 port of the STM8 SWIM bit-banged driver.
 */
#include "swim.h"

#include <sys/param.h>
#include "driver/gpio.h"
#include "esp_bit_defs.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_struct.h"

#define SWIM_GPIO 4
#define NRST_GPIO 5
#define SWIM_MASK (1U << SWIM_GPIO)
#define NRST_MASK (1U << NRST_GPIO)

static inline void set_pin_high(uint32_t mask) { REG_WRITE(GPIO_OUT_W1TS_REG, mask); }
static inline void set_pin_low(uint32_t mask) { REG_WRITE(GPIO_OUT_W1TC_REG, mask); }
static inline void pin_as_output(uint32_t mask) { REG_WRITE(GPIO_ENABLE_W1TS_REG, mask); }
static inline void pin_as_input(uint32_t mask) { REG_WRITE(GPIO_ENABLE_W1TC_REG, mask); }
static inline uint32_t read_pin(uint32_t mask) { return REG_READ(GPIO_IN_REG) & mask; }

// SWIM slow format is 8 MHz, 2 cycles high, 20 cycles low (or vice versa)
// Constants below are defined for an 80 MHz clock and scaled at runtime.
#define SHORT_PERIOD_LENGTH_80MHZ 27
#define SWIM_CLOCK_80MHZ 10
#define BIT_HALF_TIME_80MHZ (9 * SWIM_CLOCK_80MHZ)
#define BIT_TOTAL_PERIOD_LENGTH_80MHZ (22 * SWIM_CLOCK_80MHZ)

#define MICROS_TO_CYCLES(x) ((x) * cycles_per_us)

static portMUX_TYPE swim_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t TIMEOUT = 0x7FF;
static uint32_t cycles_per_us = 80;
static uint32_t swim_clock_cycles = SWIM_CLOCK_80MHZ;
static uint32_t short_period_length = SHORT_PERIOD_LENGTH_80MHZ;
static uint32_t bit_half_time = BIT_HALF_TIME_80MHZ;
static uint32_t bit_total_period_length = BIT_TOTAL_PERIOD_LENGTH_80MHZ;

static inline uint32_t get_ccount(void) {
  uint32_t ccount;
  __asm__ __volatile__("rsr %0,ccount" : "=a"(ccount));
  return ccount;
}

/** Waits until the CPU’s cycle count passes a given number. */
static inline void sync_ccount(uint32_t next) {
  while ((int32_t)(get_ccount() - next) < 0) {
  }
}

/**
 * Returns the pin to the HIGH state at the appropriate time.
 * last is the cycle count at the start of the current bit.
 * PIN must be in OUTPUT mode.
 */
static inline void finish_sync(uint32_t last) {
  sync_ccount(last + bit_total_period_length - short_period_length);
  set_pin_high(SWIM_MASK);
}

/**
 * Writes one bit. PIN must be in OUTPUT mode.
 * If the bit is 0, only writes the falling edge. The rising edge must be
 * written in the next invocation or using finish_sync().
 * * next is the cycle count at which the bit should start.
 * * prev_bit is the previous bit in case we haven't already switched to HIGH
 * * current_bit is the current bit to be written.
 */
static void write_bit_sync(uint32_t next, uint32_t prev_bit,
                           uint32_t current_bit) {
  if (!prev_bit) {
    sync_ccount(next - short_period_length);
    set_pin_high(SWIM_MASK);
  }
  sync_ccount(next);
  set_pin_low(SWIM_MASK);
  if (current_bit) {
    sync_ccount(next + short_period_length);
    set_pin_high(SWIM_MASK);
  }
}

static int read_bit(uint32_t *start);

/**
 * Sends up to 32 bits of data plus a parity bit and waits for an ack.
 * The position mask indicates how many bits to write. E.g. BIT0 writes 1 +
 * parity. BIT8 writes 9 + parity. Return value is that of read_bit() for the
 * ACK/NACK. *pnext is set to the cycles of start of the ACK bit.
 */
static int write_byte(uint32_t *pnext, uint32_t data, uint32_t mask) {
  uint32_t parity = 0;
  uint32_t next = *pnext;
  next += 200;  // Small gap to make byte boundaries visible on analyzers.
  while (mask) {
    uint32_t bit = data & mask;
    write_bit_sync(next, data & (mask << 1), bit);
    mask >>= 1;
    parity ^= !!bit;
    next += bit_total_period_length;
  }
  write_bit_sync(next, data & 1, parity);
  finish_sync(next);
  pin_as_input(SWIM_MASK);
  return read_bit(pnext);
}

/**
 * Reads one bit. PIN must be in INPUT mode.
 * In SWIM protocol a bit starts LOW.
 * Returns the bit received or -1 if no response was received.
 */
static int read_bit(uint32_t *start) {
  uint32_t timeout = TIMEOUT;
  while (read_pin(SWIM_MASK) && --timeout) {
  }
  if (!timeout) return SWIM_ERROR_READ_BIT_TIMEOUT;
  *start = get_ccount();
  sync_ccount(*start + bit_half_time);
  return read_pin(SWIM_MASK);
}

/**
 * Reads one byte from the host and sends an ACK/NACK. PIN must be in INPUT
 * mode.
 * Reads the host-bit (0), 8 data bits and the parity bit.
 * If parity matches, sends an ACK, otherwise a NACK.
 *
 * Returns -1 if the host bit is not 0 or if no bit is received after timeout.
 * Returns -2 if parity doesn’t match.
 */
static int read_byte(void) {
  uint32_t next;
  int status;
  if ((status = read_bit(&next)) != BIT4)  // target starts with 1 bit
    return status < 0 ? status : SWIM_ERROR_INVALID_TARGET_ID;
  uint32_t result = 0;
  uint32_t parity = 0;
  for (uint32_t i = 0; i < 9; i++) {
    sync_ccount(next + 18 * swim_clock_cycles);
    int bit = read_bit(&next);
    if (bit == SWIM_ERROR_READ_BIT_TIMEOUT) {
      return -(int)i - 20;
    }
    result = (result << 1) | !!bit;
    parity ^= result;
  }
  next += bit_total_period_length;
  pin_as_output(SWIM_MASK);
  write_bit_sync(next, 0, !(parity & 1));
  finish_sync(next);
  pin_as_input(SWIM_MASK);
  if (parity & 1) return SWIM_ERROR_PARITY;
  return result >> 1;
}

/**
 * Sends a command bit sequence plus all specified data bytes in succession.
 * Returns -1 if the host failed to ACK a packet within TIMEOUT.
 */
static int send_command(uint32_t cmd, size_t len, const uint8_t *data) {
  pin_as_output(SWIM_MASK);
  uint32_t next = get_ccount() + 40;
  int status = write_byte(&next, cmd, BIT3);
  if (status != BIT4) return status < 0 ? status : SWIM_ERROR_NACK;
  for (size_t i = 0; i < len; i++) {
    next = MAX(next + bit_total_period_length, get_ccount() + 40);
    pin_as_output(SWIM_MASK);
    int byte_status = write_byte(&next, data[i], BIT8);
    if (byte_status == BIT4) continue;
    if (byte_status == 0) {
      i--;
    } else if (byte_status < 0) {
      return byte_status;
    }
  }
  return 0;
}

/**
 * Encodes len and address into a 4 byte buffer.
 */
void generate_len_and_address_spec(uint8_t *dest, size_t len, uint32_t addr) {
  dest[0] = len;
  dest[1] = addr >> 16;
  dest[2] = addr >> 8;
  dest[3] = addr;
}

/**
 * Issues a read-on-the-fly command to read up to 255 bytes from a specified
 * address.
 */
int rotf(const uint8_t *len_and_address_spec, uint8_t *dest) {
  portENTER_CRITICAL_SAFE(&swim_mux);
  int status = send_command(1, 4, len_and_address_spec);
  if (status < 0) {
    portEXIT_CRITICAL_SAFE(&swim_mux);
    return status;
  }
  for (int i = 0; i < len_and_address_spec[0]; i++) {
    int result = read_byte();
    if (result < 0) {
      portEXIT_CRITICAL_SAFE(&swim_mux);
      return result;
    }
    dest[i] = result;
  }
  portEXIT_CRITICAL_SAFE(&swim_mux);
  return 0;
}

/**
 * Issues a write-on-the-fly command to write up to 255 bytes to a specified
 * address.
 *
 * Data is the buffer filled with 4 bytes using generate_len_and_address_spec
 * plus the actual data to be written.
 */
int wotf(const uint8_t *data) {
  portENTER_CRITICAL_SAFE(&swim_mux);
  int result = send_command(2, 4 + data[0], data);
  portEXIT_CRITICAL_SAFE(&swim_mux);
  return result;
}

/** Sends the SWIM command to perform a soft-reset of the device. */
int srst(void) {
  portENTER_CRITICAL_SAFE(&swim_mux);
  int result = send_command(0, 0, NULL);
  portEXIT_CRITICAL_SAFE(&swim_mux);
  return result;
}

/** Sends the SWIM activation sequence. */
int swim_entry(void) {
  pin_as_output(SWIM_MASK);
  uint32_t counter = get_ccount();

  // Initial 16us LOW
  set_pin_low(SWIM_MASK);
  counter += MICROS_TO_CYCLES(16);
  sync_ccount(counter);

  // 4 pulses at 1kHz, 4 pulses at 2kHz
  for (int i = 0; i < 16; i++) {
    if (i & 1) {
      set_pin_low(SWIM_MASK);
    } else {
      set_pin_high(SWIM_MASK);
    }
    for (int j = 0; j < 50; j++) {
      counter += (i < 8 ? MICROS_TO_CYCLES(10) : MICROS_TO_CYCLES(5));
      sync_ccount(counter);
    }
  }

  portENTER_CRITICAL_SAFE(&swim_mux);
  set_pin_high(SWIM_MASK);
  pin_as_input(SWIM_MASK);

  // Give the device time to respond.
  uint32_t timeout = MICROS_TO_CYCLES(30) / 6;
  while (read_pin(SWIM_MASK) && --timeout) {
  }
  counter = get_ccount();
  if (!timeout) {
    portEXIT_CRITICAL_SAFE(&swim_mux);
    return SWIM_ERROR_SYNC_TIMEOUT_1;
  }

  timeout = MICROS_TO_CYCLES(30) / 6;
  while (!read_pin(SWIM_MASK) && --timeout) {
  }
  int duration = get_ccount() - counter;
  if (!timeout) {
    portEXIT_CRITICAL_SAFE(&swim_mux);
    return SWIM_ERROR_SYNC_TIMEOUT_2;
  }

  portEXIT_CRITICAL_SAFE(&swim_mux);
  sync_ccount(counter + duration + 24);

  return duration;
}

/** Toggles the reset PIN. */
void reset(int on) {
  if (on == 0xFF) {
    pin_as_input(NRST_MASK);
  } else {
    pin_as_output(NRST_MASK);
    if (on) {
      set_pin_low(NRST_MASK);
    } else {
      set_pin_high(NRST_MASK);
    }
  }
}

/** On boot initialization. */
void swim_init(void) {
  cycles_per_us = esp_rom_get_cpu_ticks_per_us();
  swim_clock_cycles = cycles_per_us / 8;
  short_period_length =
      (SHORT_PERIOD_LENGTH_80MHZ * cycles_per_us) / 80;
  bit_half_time = 9 * swim_clock_cycles;
  bit_total_period_length = 22 * swim_clock_cycles;

  gpio_reset_pin(SWIM_GPIO);
  gpio_set_pull_mode(SWIM_GPIO, GPIO_PULLUP_ONLY);
  pin_as_output(SWIM_MASK);
  set_pin_high(SWIM_MASK);

  gpio_reset_pin(NRST_GPIO);
  pin_as_output(NRST_MASK);
  set_pin_high(NRST_MASK);
}
