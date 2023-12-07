/******************************************************************************
 * ctcp.c
 * ------
 * Implementation of cTCP done here. This is the only file you need to change.
 * Look at the following files for references and useful functions:
 *   - ctcp.h: Headers for this file.
 *   - ctcp_iinked_list.h: Linked list functions for managing a linked list.
 *   - ctcp_sys.h: Connection-related structs and functions, cTCP segment
 *                 definition.
 *   - ctcp_utils.h: Checksum computation, getting the current time.
 *
 *****************************************************************************/

#include "ctcp.h"
#include "ctcp_linked_list.h"
#include "ctcp_sys.h"
#include "ctcp_utils.h"
#include "ctcp_bbr.h"
#include <math.h>

#define ENABLE_STDERR_PRINT
/* #undef ENABLE_STDERR_PRINT */

#define ENABLE_BBR
/* #undef ENABLE_BBR */

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define MAX_NUM_SEND  6
#define MAX_SEG_LIFETIME  2000

/**
 * Connection state.
 *
 * Stores per-connection information such as the current sequence number,
 * unacknowledged packets, etc.
 *
 * You should add to this to store other fields you might need.
 */
typedef struct {
  int sent_time;
  long last_sent;
  #ifdef ENABLE_BBR
  int app_limited;
  #endif
  ctcp_segment_t ctcp_segment;
} to_send_ctcp_segment_t;

typedef struct {
  uint32_t last_received_ackno;
  uint32_t last_read_seqno;
  uint32_t last_sent_seqno;
  bool is_EOF_read;
  #ifdef ENABLE_BBR
  linked_list_t *to_send_segments; /* object is to_send_ctcp_segment_t */
  #endif
  linked_list_t *unack_segments; /* object is to_send_ctcp_segment_t */
} send_state_t;

typedef struct {
  uint32_t last_accepted_seqno;
  uint32_t num_truncated;
  uint32_t num_out_of_window;
  uint32_t num_invalid;
  bool is_FIN_received;
  linked_list_t* output_segments; /* object is ctcp_segment_t */
} receive_state_t;

struct ctcp_state {
  struct ctcp_state *next;  /* Next in linked list */
  struct ctcp_state **prev; /* Prev in linked list */

  conn_t *conn;             /* Connection object -- needed in order to figure out destination when sending */

  /* FIXME: Add other needed fields. */
  ctcp_config_t ctcp_config;
  long FIN_WAIT;
  send_state_t send_state;
  receive_state_t receive_state;
  #ifdef ENABLE_BBR
  FILE *bdp_fptr;
  struct bbr* bbr;
  /* uint16_t sliding_window; */
  uint32_t inflight;
  uint32_t app_limited_until;
  long min_rtt;
  uint32_t max_bw; /* unit: bit/sec */
  long last_packet_send_time;
  uint32_t last_packet_sent_size; /* byte */
  #endif
};

/**
 * Linked list of connection states. Go through this in ctcp_timer() to
 * resubmit segments and tear down connections.
 */
static ctcp_state_t *state_list;

/* FIXME: Feel free to add as many helper functions as needed. Don't repeat
          code! Helper functions make the code clearer and cleaner. */

void ctcp_send_segment(ctcp_state_t *state, to_send_ctcp_segment_t *to_send_ctcp_segment){
  if (to_send_ctcp_segment->sent_time >= MAX_NUM_SEND){
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "Maximum times of retransmit reached!\n");
    #endif
    ctcp_destroy(state);
    return;
  }
  to_send_ctcp_segment->ctcp_segment.ackno = htonl(state->receive_state.last_accepted_seqno + 1);
  to_send_ctcp_segment->ctcp_segment.flags |= TH_ACK;
  /* size_t available_bufspace = conn_bufspace(state->conn); */
  /* to_send_ctcp_segment->ctcp_segment.window = htons(min(state->ctcp_config.recv_window, available_bufspace)); */ /* flow control */
  to_send_ctcp_segment->ctcp_segment.window = htons(state->ctcp_config.recv_window);
  to_send_ctcp_segment->ctcp_segment.cksum = 0;
  to_send_ctcp_segment->ctcp_segment.cksum = cksum(&to_send_ctcp_segment->ctcp_segment, ntohs(to_send_ctcp_segment->ctcp_segment.len));

  long timestamp = current_time();
  to_send_ctcp_segment->sent_time++;
  int bytes_sent = conn_send(state->conn, &to_send_ctcp_segment->ctcp_segment, ntohs(to_send_ctcp_segment->ctcp_segment.len));
  #ifdef ENABLE_BBR
  if(to_send_ctcp_segment->sent_time == 1){
    state->last_packet_send_time = timestamp;
    state->last_packet_sent_size = ntohs(to_send_ctcp_segment->ctcp_segment.len);
  }else{
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "retransmit packet at timestamp = %ld\n", timestamp);
    #endif
  }
  #endif
  if (bytes_sent < ntohs(to_send_ctcp_segment->ctcp_segment.len)){
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "conn_send returned %d bytes instead of %d\n", bytes_sent, ntohs(to_send_ctcp_segment->ctcp_segment.len));
    #endif
    return;
  }
  if (bytes_sent == -1) {
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "Error occurs during conn_send. Terminate the connection\n");
    #endif
    ctcp_destroy(state);
    return;
  }
  #ifdef ENABLE_STDERR_PRINT
  fprintf(stderr, "SENT segment by conn_send: ");
  print_hdr_ctcp(&to_send_ctcp_segment->ctcp_segment);
  #endif
  if(to_send_ctcp_segment->sent_time == 1){ /* new packet transmission */
    state->send_state.last_sent_seqno += bytes_sent;
    #ifdef ENABLE_BBR
    /* fprintf(state->bdp_fptr, "send packet: timestamp = %ld, rtt = %ld, btlbw = %ld\n", timestamp, (long)state->bbr->rt_prop, (long)minmax_get(&state->bbr->btlbw)); */
    fprintf(state->bdp_fptr, "%ld,%u\n", timestamp, (state->bbr->rt_prop * minmax_get(&state->bbr->btlbw)) / 1000);
    /* fprintf(state->bdp_fptr, "%ld,%u\n", timestamp, state->inflight * 8); */
    fflush(state->bdp_fptr);
    #endif
  }
  to_send_ctcp_segment->last_sent = timestamp;
}

/* Selective Repeat protocol */
void ctcp_send(ctcp_state_t *state){
  if(!state)  return;
  #ifdef ENABLE_BBR
  /* retransmit segments in unack_list */
  ll_node_t *cur_node = ll_front(state->send_state.unack_segments);
  while(cur_node){
    to_send_ctcp_segment_t* unack_ctcp_segment = (to_send_ctcp_segment_t *)cur_node->object;
    assert(unack_ctcp_segment && unack_ctcp_segment->ctcp_segment.len>0 && unack_ctcp_segment->sent_time>0);
    /*
    if(unack_ctcp_segment->sent_time >= MAX_NUM_SEND){
        #ifdef ENABLE_STDERR_PRINT
        fprintf(stderr, "Maximum times of retransmit reached! Skip and remove this packet\n");
        #endif
        ll_node_t *next_node = cur_node->next;
        free(unack_ctcp_segment);
        ll_remove(state->send_state.unack_segments, cur_node);
        cur_node = next_node;
        continue;
    }
    */
    long time_diff = current_time() - unack_ctcp_segment->last_sent;
    if (time_diff > state->ctcp_config.rt_timeout){
      ctcp_send_segment(state, unack_ctcp_segment);
    }
    cur_node = cur_node->next;
  }
  /* send new segments */
  uint32_t bdp = state->bbr->rt_prop * minmax_get(&state->bbr->btlbw) / (8*1000);
  if(state->inflight >= bdp * state->bbr->cur_cwnd_gain && bdp > 0){
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "inflight >= bdp * cur_cwnd_gain. inflight = %u, bdp = %u\n",state->inflight, bdp);
    #endif
    return;
  }
  long now = current_time();
  if(now < state->bbr->next_packet_send_time){
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "current_time < next_packet_send_time. current_time = %ld, next_packet_send_time = %ld\n", now, state->bbr->next_packet_send_time);
    #endif
    return;
  }
  if(ll_length(state->send_state.to_send_segments) == 0){ /* app limit */
    state->app_limited_until = state->inflight;
    #ifdef ENABLE_STDERR_PRINT
    /* fprintf(stderr, "No new packets come. app_limited_until = %u\n", state->app_limited_until); */
    #endif
    return;
  }
  ll_node_t *front_node = ll_front(state->send_state.to_send_segments);
  to_send_ctcp_segment_t* to_send_ctcp_segment = (to_send_ctcp_segment_t *)front_node->object;
  assert(to_send_ctcp_segment && to_send_ctcp_segment->ctcp_segment.len>0 && to_send_ctcp_segment->sent_time==0);
  uint32_t segment_last_seqno = ntohl(to_send_ctcp_segment->ctcp_segment.seqno) + max(ntohs(to_send_ctcp_segment->ctcp_segment.len)-sizeof(ctcp_segment_t)-1, 0);
  uint32_t last_allowable_seqno = state->ctcp_config.send_window + (state->send_state.last_received_ackno==0?0:state->send_state.last_received_ackno-1);
  if(segment_last_seqno > last_allowable_seqno){
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "state->ctcp_config.send_window = %d\n", state->ctcp_config.send_window);
    fprintf(stderr, "ctcp_send failed to send anything as segment_last_seqno = %d and last_allowable_seqno = %d\n", segment_last_seqno, last_allowable_seqno);
    #endif
    return;
  }
  if(state->app_limited_until > 0){
    to_send_ctcp_segment->app_limited = true;
  }
  ctcp_send_segment(state, to_send_ctcp_segment);
  assert(state->last_packet_sent_size == ntohs(to_send_ctcp_segment->ctcp_segment.len));
  #ifdef ENABLE_STDERR_PRINT
  fprintf(stderr, "state->inflight += %u\n", state->last_packet_sent_size);
  #endif
  state->inflight += state->last_packet_sent_size;
  /* assert(minmax_get(&state->bbr->btlbw) > 0); */
  if(minmax_get(&state->bbr->btlbw)==0 || state->inflight >= state->bbr->cur_cwnd_gain*(minmax_get(&state->bbr->btlbw)*state->bbr->rt_prop/(1000*8))){
    state->bbr->next_packet_send_time = now;
  }else{
    state->bbr->next_packet_send_time = state->last_packet_send_time + (int)(state->last_packet_sent_size*8*1000 / (state->bbr->cur_pacing_gain * minmax_get(&state->bbr->btlbw)));
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "ctcp send pahse!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
    fprintf(stderr, "state->bbr->next_packet_send_time is set ahead to %ld\n", state->bbr->next_packet_send_time);
    fprintf(stderr, "last_packet_sent_size = %u, cur_pacing_gain = %f, btlbw = %u\n", state->last_packet_sent_size, state->bbr->cur_pacing_gain, minmax_get(&state->bbr->btlbw));
    fprintf(stderr, "last_packet_send_time = %ld, added increment = %f\n", state->last_packet_send_time, state->last_packet_sent_size*8*1000 / (state->bbr->cur_pacing_gain * minmax_get(&state->bbr->btlbw)));
    #endif
  }
  /* move to unack list */
  int bytes_payload = ntohs(to_send_ctcp_segment->ctcp_segment.len)-sizeof(ctcp_segment_t);
  to_send_ctcp_segment_t* unack_ctcp_segment = (to_send_ctcp_segment_t *)calloc(sizeof(to_send_ctcp_segment_t) + bytes_payload, 1);
  memcpy(unack_ctcp_segment, to_send_ctcp_segment, sizeof(to_send_ctcp_segment_t) + bytes_payload);
  ll_add(state->send_state.unack_segments, unack_ctcp_segment);
  free(to_send_ctcp_segment);
  ll_remove(state->send_state.to_send_segments, front_node); /* free to_send_ctcp_segment */
  /*
  while(ll_length(state->send_state.to_send_segments) > 0 && state->sliding_window < state->ctcp_config.send_window){
    ll_add(state->send_state.unack_segments, ll_front(state->send_state.to_send_segments));
    ll_node_t* front_node = ll_front(state->send_state.to_send_segments);
    to_send_ctcp_segment_t* to_send_ctcp_segment = (to_send_ctcp_segment_t *)front_node->object;
    ll_remove(state->send_state.to_send_segments, ll_front(state->send_state.to_send_segments));
    state->sliding_window += ntohs(to_send_ctcp_segment->ctcp_segment.len) - sizeof(ctcp_segment_t);
  }
  clean_sliding_window(state);
  */
  #else
  ll_node_t *cur_node = ll_front(state->send_state.unack_segments);
  while(cur_node){
    to_send_ctcp_segment_t* to_send_ctcp_segment = (to_send_ctcp_segment_t *)cur_node->object;
    assert(to_send_ctcp_segment && to_send_ctcp_segment->ctcp_segment.len>0);
    uint32_t segment_last_seqno = ntohl(to_send_ctcp_segment->ctcp_segment.seqno) + max(ntohs(to_send_ctcp_segment->ctcp_segment.len)-sizeof(ctcp_segment_t)-1, 0);
    uint32_t last_allowable_seqno = state->ctcp_config.send_window + (state->send_state.last_received_ackno==0?0:state->send_state.last_received_ackno-1);
    if(segment_last_seqno > last_allowable_seqno){
      #ifdef ENABLE_STDERR_PRINT
      fprintf(stderr, "state->ctcp_config.send_window = %d\n", state->ctcp_config.send_window);
      fprintf(stderr, "ctcp_send failed to send anything as segment_last_seqno = %d and last_allowable_seqno = %d\n", segment_last_seqno, last_allowable_seqno);
      #endif
      return;
    }
    if(to_send_ctcp_segment->sent_time == 0){ /* can be transmitted if it's new */
      ctcp_send_segment(state, to_send_ctcp_segment);
    }else if(cur_node == ll_front(state->send_state.unack_segments)){ /* can be retransmitted if it's the first segment */
      long time_diff = current_time() - to_send_ctcp_segment->last_sent;
      if (time_diff > state->ctcp_config.rt_timeout)  ctcp_send_segment(state, to_send_ctcp_segment);
    }
    cur_node = cur_node->next;
  }
  #endif
}

void ctcp_send_ack_segment(ctcp_state_t *state, size_t cur_window_size){
  ctcp_segment_t ctcp_segment;

  ctcp_segment.seqno = htonl(0); /* seqno does not matter */
  ctcp_segment.ackno = htonl(state->receive_state.last_accepted_seqno + 1);
  ctcp_segment.len = sizeof(ctcp_segment_t);
  ctcp_segment.flags |= TH_ACK;
  /* ctcp_segment.window = htons(min(state->ctcp_config.recv_window, cur_window_size)); */ /* flow control */
  ctcp_segment.window = htons(state->ctcp_config.recv_window);
  ctcp_segment.cksum = 0;
  ctcp_segment.cksum = cksum(&ctcp_segment, sizeof(ctcp_segment_t));

  long now = current_time();
  conn_send(state->conn, &ctcp_segment, sizeof(ctcp_segment_t));
  #ifdef ENABLE_STDERR_PRINT
  fprintf(stderr, "SENT ACK by conn_send at timestamp = %ld: ", now);
  print_hdr_ctcp(&ctcp_segment);
  #endif
}

void clean_list(linked_list_t * segments_list){
  ll_node_t *cur = segments_list->head;
  while (cur != NULL){
    free(cur->object);
    cur = cur->next;
  }
}

void clean_acked_segments(ctcp_state_t *state){
  #ifdef ENABLE_BBR 
  long now = current_time();
  #endif
  while(ll_length(state->send_state.unack_segments) > 0) {
    ll_node_t* front_node = ll_front(state->send_state.unack_segments);
    to_send_ctcp_segment_t *unack_ctcp_segment = (to_send_ctcp_segment_t *)front_node->object;
    int num_data_bytes = ntohs(unack_ctcp_segment->ctcp_segment.len) - sizeof(ctcp_segment_t);
    uint32_t segment_last_seqno = ntohl(unack_ctcp_segment->ctcp_segment.seqno) + max(num_data_bytes-1, 0);
    if (segment_last_seqno < state->send_state.last_received_ackno) {
      #ifdef ENABLE_STDERR_PRINT
      fprintf(stderr, "Sent segment with seqno_of_last_byte: %d has been acknowledged\n", segment_last_seqno);
      #endif
      #ifdef ENABLE_BBR 
      long rtt = now - unack_ctcp_segment->last_sent;
      rtt = max(rtt, 1);
      assert(rtt > 0);
      if(state->min_rtt == 0 || rtt < state->min_rtt){ /* update rtt */
        state->min_rtt = rtt;
      }
      bool filter_expired = (now - state->bbr->min_rtt_stamp) > bbr_min_rtt_win_sec * 1000;
      if(!state->bbr->rt_prop || (state->bbr->rt_prop && rtt<state->bbr->rt_prop) || filter_expired){
        state->bbr->rt_prop = rtt;
        state->bbr->min_rtt_stamp = now;
      }
      state->bbr->num_rtt++;
      if(state->bbr->num_rtt == 1 || !unack_ctcp_segment->app_limited){ /* update bw */
        uint32_t bw = ntohs(unack_ctcp_segment->ctcp_segment.len)*8*1000 / rtt;
        minmax_running_max(&state->bbr->btlbw, bbr_bw_rtts, state->bbr->num_rtt, bw);
        if(state->max_bw < bw || state->max_bw == 0){
          state->max_bw = bw;
          state->bbr->full_bw_cnt = 0;
        }else{
          state->bbr->full_bw_cnt++;
        }
      }
      /*
      fprintf(state->bdp_fptr, "receive sent packet ack: timestamp = %ld, rtt = %ld, btlbw = %ld\n", now, (long)state->bbr->rt_prop, (long)minmax_get(&state->bbr->btlbw));
      fflush(state->bdp_fptr);
      */
      if(state->app_limited_until > 0){
        state->app_limited_until -= ntohs(unack_ctcp_segment->ctcp_segment.len);
        if(state->app_limited_until <= 0){
          state->app_limited_until = 0;
        }
      }
      state->inflight -= ntohs(unack_ctcp_segment->ctcp_segment.len);
      check_bbr_state(state->bbr, state->inflight, now, filter_expired); /* check and change BBR mode */
      if(minmax_get(&state->bbr->btlbw)==0 || state->inflight >= state->bbr->cur_cwnd_gain*(minmax_get(&state->bbr->btlbw)*state->bbr->rt_prop/(1000*8))){
        state->bbr->next_packet_send_time = now;
      }else{
        state->bbr->next_packet_send_time = state->last_packet_send_time + (int)(state->last_packet_sent_size*8*1000 / (state->bbr->cur_pacing_gain * minmax_get(&state->bbr->btlbw)));
        #ifdef ENABLE_STDERR_PRINT
        fprintf(stderr, "clean ACK pahse!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        fprintf(stderr, "state->bbr->next_packet_send_time is set ahead to %ld\n", state->bbr->next_packet_send_time);
        fprintf(stderr, "last_packet_sent_size = %u, cur_pacing_gain = %f, btlbw = %u\n", state->last_packet_sent_size, state->bbr->cur_pacing_gain, minmax_get(&state->bbr->btlbw));
        fprintf(stderr, "last_packet_send_time = %ld, added increment = %f\n", state->last_packet_send_time, state->last_packet_sent_size*8*1000 / (state->bbr->cur_pacing_gain * minmax_get(&state->bbr->btlbw)));
        #endif
      }
      #endif
      free(unack_ctcp_segment);
      ll_remove(state->send_state.unack_segments, front_node);
    }else{
      #ifdef ENABLE_STDERR_PRINT
      fprintf(stderr, "Sent segment with seqno_of_last_byte: %d has not been acknowledged, as current last_received_ackno = %d\n", segment_last_seqno, state->send_state.last_received_ackno);
      #endif
      return;
    }
  }
}

ctcp_state_t *ctcp_init(conn_t *conn, ctcp_config_t *cfg) {
  /* Connection could not be established. */
  if (conn == NULL) {
    return NULL;
  }

  /* Established a connection. Create a new state and update the linked list
     of connection states. */
  ctcp_state_t *state = calloc(sizeof(ctcp_state_t), 1);
  state->next = state_list;
  state->prev = &state_list;
  if (state_list)
    state_list->prev = &state->next;
  state_list = state;

  /* Set fields. */
  state->conn = conn;
  /* FIXME: Do any other initialization here. */
  state->ctcp_config.recv_window = cfg->recv_window;
  state->ctcp_config.send_window = cfg->send_window;
  state->ctcp_config.timer = cfg->timer;
  state->ctcp_config.rt_timeout = cfg->rt_timeout;
  #ifdef ENABLE_STDERR_PRINT
  fprintf(stderr, "ctcp_config->recv_window: %d\n", state->ctcp_config.recv_window );
  fprintf(stderr, "ctcp_config->send_window: %d\n", state->ctcp_config.send_window );
  fprintf(stderr, "ctcp_config->timer: %d\n", state->ctcp_config.timer );
  fprintf(stderr, "ctcp_config->rt_timeout: %d\n", state->ctcp_config.rt_timeout );
  #endif
  free(cfg);

  state->FIN_WAIT = 0;

  state->send_state.last_received_ackno = 0;
  state->send_state.last_read_seqno = 0;
  state->send_state.last_sent_seqno = 0;
  state->send_state.is_EOF_read = false;
  #ifdef ENABLE_BBR
  state->bdp_fptr  = fopen("bdp.txt","w");
  state->send_state.to_send_segments = ll_create();
  #endif
  state->send_state.unack_segments = ll_create();

  state->receive_state.last_accepted_seqno = 0;
  state->receive_state.num_truncated = 0;
  state->receive_state.num_out_of_window = 0;
  state->receive_state.num_invalid = 0;
  state->receive_state.is_FIN_received = false;
  state->receive_state.output_segments = ll_create();

  #ifdef ENABLE_BBR
  /* state->sliding_window = 0; */
  state->inflight = 0;
  state->app_limited_until = 0;
  state->min_rtt = 0;
  state->max_bw = 0;
  state->last_packet_send_time = 0;
  state->last_packet_sent_size = 0;
  state->bbr = bbr_init();
  #endif

  return state;
}

void ctcp_destroy(ctcp_state_t *state) {
  if(!state){
    end_client();
    return;
  }
  #ifdef ENABLE_STDERR_PRINT
  fprintf(stderr, "Number of truncated segments: %u\n", state->receive_state.num_truncated);
  fprintf(stderr, "Number of out_of_window segments: %u\n", state->receive_state.num_out_of_window);
  fprintf(stderr, "Number of invalid_cksum segments: %u\n", state->receive_state.num_invalid);
  #endif

  /* Update linked list. */
  if (state->next)
    state->next->prev = state->prev;

  *state->prev = state->next;
  conn_remove(state->conn);

  /* FIXME: Do any other cleanup here. */
  int len = ll_length(state->send_state.unack_segments);
  if (len>0){
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "There are %d segments never acknowledged!\n", len);
    #endif
  }
  clean_list(state->send_state.unack_segments);
  ll_destroy(state->send_state.unack_segments);
  #ifdef ENABLE_BBR
  len = ll_length(state->send_state.to_send_segments);
  if (len>0){
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "There are %d segments never sent out!\n", len);
    #endif
  }
  clean_list(state->send_state.to_send_segments);
  ll_destroy(state->send_state.to_send_segments);
  #endif
  len = ll_length(state->receive_state.output_segments);
  if (len>0){
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "There are %d segments never output!\n", len);
    #endif
  }
  clean_list(state->receive_state.output_segments);
  ll_destroy(state->receive_state.output_segments);

  free(state);
  end_client();
}

void ctcp_read(ctcp_state_t *state) {
  /* FIXME */
  char buf[MAX_SEG_DATA_SIZE+1];
  int bytes_read;
  if (state->send_state.is_EOF_read){
      #ifdef ENABLE_STDERR_PRINT
      /* fprintf(stderr, "Already read EOF! Can not send data any more!\n"); */
      #endif
      return;
  }
  while((bytes_read = conn_input(state->conn, buf, MAX_SEG_DATA_SIZE)) > 0){
    #ifdef ENABLE_STDERR_PRINT
    buf[bytes_read] = 0;
    fprintf(stderr, "Read %d bytes: %s\n", bytes_read, buf);
    #endif
    to_send_ctcp_segment_t *segment = (to_send_ctcp_segment_t *)calloc(sizeof(to_send_ctcp_segment_t) + bytes_read, 1);
    assert(segment);
    segment->ctcp_segment.len = htons((uint16_t) sizeof(ctcp_segment_t) + bytes_read);
    segment->ctcp_segment.seqno = htonl(state->send_state.last_read_seqno + 1);
    state->send_state.last_read_seqno += bytes_read;
    memcpy(segment->ctcp_segment.data, buf, bytes_read);
    #ifdef ENABLE_BBR
    ll_add(state->send_state.to_send_segments, segment);
    #else
    ll_add(state->send_state.unack_segments, segment);
    #endif
  }
  if (bytes_read == -1) {
    /* EOF or error, send a FIN segment */
    state->send_state.is_EOF_read = true;
    to_send_ctcp_segment_t *fin_segment = (to_send_ctcp_segment_t *)calloc(sizeof(to_send_ctcp_segment_t), 1);
    assert(fin_segment);
    fin_segment->ctcp_segment.len = htons((uint16_t) sizeof(ctcp_segment_t));
    fin_segment->ctcp_segment.seqno = htonl(state->send_state.last_read_seqno + 1);
    fin_segment->ctcp_segment.flags |= TH_FIN;
    #ifdef ENABLE_BBR
    ll_add(state->send_state.to_send_segments, fin_segment);
    #else
    ll_add(state->send_state.unack_segments, fin_segment);
    #endif
  }
  ctcp_send(state);
}

void ctcp_receive(ctcp_state_t *state, ctcp_segment_t *segment, size_t len) {
  if (len < ntohs(segment->len)){
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "Truncated segment. Drop\n");
    print_hdr_ctcp(segment);
    #endif
    free(segment);
    state->receive_state.num_truncated++;
    return;
  }
  uint16_t received_cksum = segment->cksum;
  segment->cksum = 0;
  uint16_t computed_cksum = cksum(segment, ntohs(segment->len));
  segment->cksum = received_cksum;
  if (received_cksum != computed_cksum) {
    #ifdef ENABLE_STDERR_PRINT
    fprintf(stderr, "Invalid cksum! received_cksum=%x, computed_cksum=%x. Drop\n", received_cksum, computed_cksum);
    print_hdr_ctcp(segment);
    #endif
    free(segment);
    state->receive_state.num_invalid++;
    return;
  }

  int num_data_bytes = ntohs(segment->len) - sizeof(ctcp_segment_t);
  if (num_data_bytes || (segment->flags & TH_FIN)){
    uint32_t segment_last_seqno = ntohl(segment->seqno) + max(num_data_bytes-1, 0);
    uint32_t first_allowable_seqno = state->receive_state.last_accepted_seqno + 1;
    uint32_t last_allowable_seqno = state->receive_state.last_accepted_seqno + state->ctcp_config.recv_window;
    if ((segment_last_seqno > last_allowable_seqno) || (ntohl(segment->seqno) < first_allowable_seqno)) {
      #ifdef ENABLE_STDERR_PRINT
      fprintf(stderr, "First_allowable_seqno: %d, Last_allowable_seqno: %d\n", first_allowable_seqno, last_allowable_seqno);
      fprintf(stderr, "Out of window segment. Drop\n");
      print_hdr_ctcp(segment);
      #endif
      free(segment);
      /* size_t available_bufspace = conn_bufspace(state->conn); */
      /* ctcp_send_ack_segment(state, available_bufspace); */ /* flow control */
      ctcp_send_ack_segment(state, 0);
      state->receive_state.num_out_of_window++;
      return;
    }
  }

  #ifdef ENABLE_STDERR_PRINT
  fprintf(stderr, "Received a valid segment with %d bytes\n", num_data_bytes);
  print_hdr_ctcp(segment);
  #endif

  if (segment->flags & TH_ACK) {
    state->send_state.last_received_ackno = ntohl(segment->ackno);
    /* state->ctcp_config.send_window = ntohs(segment->window); */ /* flow control */
    clean_acked_segments(state);
  }

  if (num_data_bytes || (segment->flags & TH_FIN)){
    if (ll_length(state->receive_state.output_segments)==0){
      assert(ll_add(state->receive_state.output_segments, segment));
    }else{
      ll_node_t* cur_node = ll_front(state->receive_state.output_segments);
      ll_node_t* next_node = cur_node->next;
      ctcp_segment_t* cur_ctcp_segment = (ctcp_segment_t*)cur_node->object;
      ctcp_segment_t* next_ctcp_segment = next_node?(ctcp_segment_t*)next_node->object:NULL;
      if (ntohl(segment->seqno) < ntohl(cur_ctcp_segment->seqno)){
        assert(ll_add_front(state->receive_state.output_segments, segment));
      }else{
        while(cur_node){
          cur_ctcp_segment = (ctcp_segment_t*)cur_node->object;
          next_ctcp_segment = next_node?(ctcp_segment_t*)next_node->object:NULL;
          if (ntohl(segment->seqno) == ntohl(cur_ctcp_segment->seqno)){
            free(segment);
            break;
          }
          if (next_node==NULL || (next_node && ntohl(cur_ctcp_segment->seqno) < ntohl(segment->seqno) && ntohl(segment->seqno) < ntohl(next_ctcp_segment->seqno))){
            assert(ll_add_after(state->receive_state.output_segments, cur_node, segment));
            break;
          }
          cur_node = next_node;
          next_node = cur_node?cur_node->next:NULL;
        }
      }
    }
    ctcp_output(state);
  }else{
    free(segment);
  }
  /* ctcp_output(state); */
  /* clean_acked_segments(state); */
}

void ctcp_output(ctcp_state_t *state) {
  /* FIXME */
  int num_output = 0;
  size_t available_bufspace = conn_bufspace(state->conn);
  while(ll_length(state->receive_state.output_segments)>0){
    ll_node_t* front_node = ll_front(state->receive_state.output_segments);
    ctcp_segment_t* ctcp_segment = (ctcp_segment_t*) front_node->object;
    int num_data_bytes = ntohs(ctcp_segment->len) - sizeof(ctcp_segment_t);
    /* available_bufspace = conn_bufspace(state->conn); */
    if(ntohl(ctcp_segment->seqno) != state->receive_state.last_accepted_seqno + 1){ /* There's a hole */
      /* ctcp_send_ack_segment(state, available_bufspace); */
      return;
    }
    if(num_data_bytes){
      available_bufspace = conn_bufspace(state->conn);
      if(available_bufspace < num_data_bytes){  /* insufficient buffer */
        /* ctcp_send_ack_segment(state, available_bufspace); */
        return;
      }
      int ret = conn_output(state->conn, ctcp_segment->data, num_data_bytes);
      if (ret==-1){
        #ifdef ENABLE_STDERR_PRINT
        fprintf(stderr, "Error occurs during conn_output(). Tear down connection\n");
        #endif
        ctcp_destroy(state);
        return;
      }
      assert(ret == num_data_bytes);
      num_output++;
      state->receive_state.last_accepted_seqno += num_data_bytes;
      ctcp_send_ack_segment(state, available_bufspace);
    }
    if ((!state->receive_state.is_FIN_received) && (ctcp_segment->flags & TH_FIN)) {  /* FIN */
      state->receive_state.is_FIN_received = true;
      #ifdef ENABLE_STDERR_PRINT
      fprintf(stderr, "FIN received\n");
      #endif
      state->receive_state.last_accepted_seqno++;
      conn_output(state->conn, ctcp_segment->data, 0);
      num_output++;
      ctcp_send_ack_segment(state, available_bufspace);
    }
    free(ctcp_segment);
    ll_remove(state->receive_state.output_segments, front_node);
  }
  /* if(num_output)  ctcp_send_ack_segment(state, available_bufspace); */
}

void ctcp_timer() {
  /* FIXME */
  ctcp_state_t* cur_state = state_list;
  while(cur_state){
    ctcp_output(cur_state);
    ctcp_send(cur_state);
    #ifdef ENABLE_BBR
    long now = current_time();
    if((now - cur_state->bbr->min_rtt_stamp) > bbr_min_rtt_win_sec * 1000){
      #ifdef ENABLE_STDERR_PRINT
      fprintf(stderr, "ctcp_time phase !!!!!!!!!!!!!!!BBR Mode change to BBR_PROBE_RTT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
      #endif
		  cur_state->bbr->mode = BBR_PROBE_RTT;
		  cur_state->bbr->cur_cwnd_gain = 1;
		  cur_state->bbr->cur_pacing_gain = 0.75;
		  cur_state->bbr->probe_rtt_done_stamp = 0;
    }
    if(cur_state->receive_state.is_FIN_received && cur_state->send_state.is_EOF_read && ll_length(cur_state->send_state.to_send_segments)==0 && ll_length(cur_state->send_state.unack_segments)==0 && ll_length(cur_state->receive_state.output_segments)==0){
    #else
    if(cur_state->receive_state.is_FIN_received && cur_state->send_state.is_EOF_read && ll_length(cur_state->send_state.unack_segments)==0 && ll_length(cur_state->receive_state.output_segments)==0){
    #endif
      if (cur_state->FIN_WAIT==0){
        #ifdef ENABLE_STDERR_PRINT
        fprintf(stderr, "Ready to tear down connection. Wait for FIN_WAIT...\n");
        #endif
        cur_state->FIN_WAIT = current_time();
      } else if(current_time() - cur_state->FIN_WAIT > 2 * MAX_SEG_LIFETIME) {
        #ifdef ENABLE_STDERR_PRINT
        fprintf(stderr, "Now close down the connection.\n");
        #endif
        ctcp_destroy(cur_state);
      }
    }
    cur_state = cur_state->next;
  }
}
