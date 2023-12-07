#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include "ctcp.h"
#include "ctcp_linked_list.h"
#include "ctcp_sys.h"
#include "ctcp_utils.h"

#define MAX_FULL_BW_ROUND 3
#define CYCLE_LEN 8	

/* A single data point for our parameterized min-max tracker */
struct minmax_sample {
	uint32_t t;	/* time measurement was taken */
	uint32_t	v;	/* value measured */
};

/* State for the parameterized min-max tracker */
struct minmax {
	struct minmax_sample s[3];
};

static inline uint32_t minmax_get(const struct minmax *m)
{
	return m->s[0].v;
}

static inline uint32_t minmax_reset(struct minmax *m, uint32_t t, uint32_t meas)
{
	struct minmax_sample val = { .t = t, .v = meas };

	m->s[2] = m->s[1] = m->s[0] = val;
	return m->s[0].v;
}

uint32_t minmax_running_max(struct minmax *m, uint32_t win, uint32_t t, uint32_t meas);
uint32_t minmax_running_min(struct minmax *m, uint32_t win, uint32_t t, uint32_t meas);

struct bbr {
	unsigned int mode;
	uint32_t	rt_prop; /* ms */
	struct minmax btlbw; /* unit: bit/sec */
	int num_rtt;
	long min_rtt_stamp; /* time when min_rtt was updated */
	long	probe_rtt_done_stamp;   /* end time for BBR_PROBE_RTT mode */
  long next_packet_send_time;	/* next packet send time*/
	int cycle_idx; /* index of the current gain cycle */
	long cycle_start_time; /* time when the current cycle started */
	float cur_pacing_gain; /* tracks the current pacing gain value */
	float cur_cwnd_gain; /* tracks the current cwnd gain value */
	int full_bw_cnt; /* N rounds w/o bw growth -> pipe full */
};

 enum bbr_mode {
 	BBR_STARTUP,	/* ramp up sending rate rapidly to fill pipe */
 	BBR_DRAIN,	/* drain any queue created during startup */
 	BBR_PROBE_BW,	/* discover, share bw: pace around estimated bw */
 	BBR_PROBE_RTT,	/* cut cwnd to min to probe min_rtt */
 };

 static const float bbr_high_gain  = 2.885;	/* 2/ln(2) */
 static const float bbr_drain_gain = (float)1/2.885;	/* 1/high_gain */
 static const float bbr_cwnd_gain  = 2;	/* gain for steady-state cwnd */
 static const int bbr_bw_rtts = 10; /* win len of bw filter (in rounds) */
 static const int bbr_min_rtt_win_sec = 10;	/* min RTT filter window (in sec) */
static const int bbr_probe_rtt_mode_ms = 200;	 /* min ms at cwnd=4 in BBR_PROBE_RTT */
 /* The pacing_gain values for the PROBE_BW gain cycle: */
 static const float bbr_pacing_gain[] = { 1.25, 0.75, 1, 1, 1, 1, 1, 1 };

 struct bbr* bbr_init();
 void check_bbr_state(struct bbr* bbr, uint32_t inflight, long cur_time, bool rtt_expired);