#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "ctcp.h"
#include "ctcp_linked_list.h"
#include "ctcp_sys.h"
#include "ctcp_utils.h"
#include "ctcp_bbr.h"

#define ENABLE_STDERR_PRINT
/* #undef ENABLE_STDERR_PRINT */

static uint32_t minmax_subwin_update(struct minmax *m, uint32_t win,
				const struct minmax_sample *val)
{
	uint32_t dt = val->t - m->s[0].t;

	if (dt > win) {
		/*
		 * Passed entire window without a new val so make 2nd
		 * choice the new val & 3rd choice the new 2nd choice.
		 * we may have to iterate this since our 2nd choice
		 * may also be outside the window (we checked on entry
		 * that the third choice was in the window).
		 */
		m->s[0] = m->s[1];
		m->s[1] = m->s[2];
		m->s[2] = *val;
		if (val->t - m->s[0].t > win) {
			m->s[0] = m->s[1];
			m->s[1] = m->s[2];
			m->s[2] = *val;
		}
	} else if (m->s[1].t == m->s[0].t && dt > win/4) {
		/*
		 * We've passed a quarter of the window without a new val
		 * so take a 2nd choice from the 2nd quarter of the window.
		 */
		m->s[2] = m->s[1] = *val;
	} else if (m->s[2].t == m->s[1].t && dt > win/2) {
		/*
		 * We've passed half the window without finding a new val
		 * so take a 3rd choice from the last half of the window
		 */
		m->s[2] = *val;
	}
	return m->s[0].v;
}

uint32_t minmax_running_max(struct minmax *m, uint32_t win, uint32_t t, uint32_t meas)
{
	struct minmax_sample val = { .t = t, .v = meas };

	if (val.v >= m->s[0].v ||	  /* found new max? */
	    val.t - m->s[2].t > win)	  /* nothing left in window? */
		return minmax_reset(m, t, meas);  /* forget earlier samples */

	if (val.v >= m->s[1].v)
		m->s[2] = m->s[1] = val;
	else if (val.v >= m->s[2].v)
		m->s[2] = val;

	return minmax_subwin_update(m, win, &val);
}

bool bbr_is_next_cycle_phase(struct bbr* bbr, uint32_t inflight, long cur_time){
	bool is_full_length = (cur_time - bbr->cycle_start_time) >= bbr->rt_prop;
	if(bbr->cur_pacing_gain == 1){
		return is_full_length;
	}
	if(bbr->cur_pacing_gain > 1){
		return is_full_length && inflight >= bbr->cur_pacing_gain*(minmax_get(&bbr->btlbw)*bbr->rt_prop/(8*1000));
	}
	return is_full_length || minmax_get(&bbr->btlbw)*bbr->rt_prop/(8*1000);
}

void check_bbr_state(struct bbr* bbr, uint32_t inflight, long cur_time, bool rtt_expired){
	if(rtt_expired && bbr->mode != BBR_PROBE_RTT){
		#ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "BBR Mode change to BBR_PROBE_RTT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    #endif
		bbr->mode = BBR_PROBE_RTT;
		bbr->cur_cwnd_gain = 1;
		bbr->cur_pacing_gain = 0.75;
		bbr->probe_rtt_done_stamp = 0;
	}
	switch (bbr->mode) {
        case BBR_STARTUP:
            if (bbr->full_bw_cnt > MAX_FULL_BW_ROUND) { /* bbr_full_bw_reached */
								#ifdef ENABLE_STDERR_PRINT
    						fprintf(stderr, "BBR Mode change to BBR_DRAIN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    						#endif
                bbr->mode = BBR_DRAIN;
 								bbr->cur_pacing_gain = bbr_drain_gain;	/* pace slow to drain */
 								bbr->cur_cwnd_gain = bbr_high_gain;	/* maintain cwnd */
            }
            break;
        case BBR_DRAIN:
            if (inflight <= minmax_get(&bbr->btlbw)*bbr->rt_prop/(1000*8)) { /* inflight <= BDP */
								/* bbr_reset_probe_bw_mode */
								#ifdef ENABLE_STDERR_PRINT
    						fprintf(stderr, "BBR Mode change to BBR_PROBE_BW!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    						#endif
                bbr->mode = BBR_PROBE_BW;
								bbr->cur_cwnd_gain = bbr_cwnd_gain;
								srand(time(NULL));
								bbr->cycle_idx = rand() % CYCLE_LEN;	/* random index */
								bbr->cur_pacing_gain = bbr_pacing_gain[bbr->cycle_idx];
								bbr->cycle_start_time = cur_time;
            }
            break;

        case BBR_PROBE_BW:
						if(bbr_is_next_cycle_phase(bbr, inflight, cur_time)){
									/* update cycle of PROBE_BW */
									bbr->cycle_idx = (bbr->cycle_idx + 1) % CYCLE_LEN;
									bbr->cur_pacing_gain = bbr_pacing_gain[bbr->cycle_idx];
									bbr->cycle_start_time = cur_time;
						}
            break;

        case BBR_PROBE_RTT:
            if (bbr->probe_rtt_done_stamp == 0) {
                bbr->probe_rtt_done_stamp = cur_time + bbr_probe_rtt_mode_ms;
            }else if(bbr->probe_rtt_done_stamp > 0){
							if (cur_time >= bbr->probe_rtt_done_stamp) { /* bbr_probe_rtt_done */
 								bbr->min_rtt_stamp = cur_time;
 								if (bbr->full_bw_cnt > MAX_FULL_BW_ROUND){ /* bbr_full_bw_reached */
									/* bbr_reset_probe_bw_mode */
									#ifdef ENABLE_STDERR_PRINT
    							fprintf(stderr, "BBR Mode change to BBR_PROBE_BW!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    							#endif
 									bbr->mode = BBR_PROBE_BW;
									bbr->cur_cwnd_gain = bbr_cwnd_gain;
									srand(time(NULL));
									bbr->cycle_idx = rand() % CYCLE_LEN;	/* random index */
									bbr->cur_pacing_gain = bbr_pacing_gain[bbr->cycle_idx];
									bbr->cycle_start_time = cur_time;	
								}else{
									/* bbr_reset_startup_mode */
									#ifdef ENABLE_STDERR_PRINT
    							fprintf(stderr, "BBR Mode change to BBR_STARTUP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    							#endif
									bbr->mode = BBR_STARTUP;
 									bbr->cur_pacing_gain = bbr_high_gain;
 									bbr->cur_cwnd_gain	 = bbr_high_gain;
								}
 							}
						}
            break;
    }
}

struct bbr* bbr_init(){
	struct bbr* bbr = malloc(sizeof(struct bbr));
	bbr->mode = BBR_STARTUP;
	bbr->rt_prop = 0;
	minmax_reset(&bbr->btlbw, 0, 0);
	bbr->num_rtt = 0;
	/* minmax_running_max(&bbr->btlbw, bbr_bw_rtts, bbr->num_rtt, 1440*8*1000); */
	bbr->min_rtt_stamp = 0;
	bbr->probe_rtt_done_stamp = 0;
	bbr->next_packet_send_time = 0;
	bbr->cycle_idx = 0;
	bbr->cycle_start_time = 0;
	bbr->cur_pacing_gain = bbr_high_gain;
	bbr->cur_cwnd_gain = bbr_high_gain;
	bbr->full_bw_cnt = 0;
	return bbr;
}