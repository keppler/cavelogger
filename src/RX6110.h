#ifndef __RX6110_H
#define __RX6110_H

void RTC_setup(void);
uint8_t RTC_init(void);		/* 0=ok, 1=error */

struct RTC_ts {
	uint8_t ts_sec;
	uint8_t ts_min;
	uint8_t ts_hour;
	uint8_t ts_wday;
	uint8_t ts_mday;
	uint8_t ts_mon;
	uint8_t ts_year;
};

void RTC_get(struct RTC_ts *ts);
void RTC_reset();
void RTC_setTimer(uint16_t interval);
void RTC_set(const struct RTC_ts *ts);
uint8_t RTC_powerLoss();

#endif /* !__RX6110_H */
