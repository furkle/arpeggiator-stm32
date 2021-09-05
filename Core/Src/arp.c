/*
 * arp.c
 *
 *  Created on: Sep 2, 2021
 *      Author: furkle
 */

#include "adc.h"
#include "arp.h"

extern uint32_t adc1_avg;
extern uint32_t adc2_avg;

void arp_init(ArpSettings* settings) {
  settings->bpm = 120;
  // Default, changed by code
  settings->beat_division = QUARTER_BEAT;
  settings->direction = ARP_UP;
  // Default, changed by pot
  settings->interval = PERFECT_FIFTH;
  settings->last_seen_time = 0;
  settings->notes_played_this_iter = 0;
  settings->root_note_num = 0;
  settings->notes_to_play = 4;
}

void update_bpm(ArpSettings* settings) {
	uint32_t ms_elapsed;
	if (settings->last_seen_time) {
		ms_elapsed = settings->last_seen_time;
		settings->last_seen_time = HAL_GetTick();
		ms_elapsed = settings->last_seen_time - ms_elapsed;
	} else {
		settings->last_seen_time = HAL_GetTick();
		return;
	}

	settings->bpm = (uint16_t)(60.0 * ((double)ms_elapsed / 1000.0));
}

void arpeggiate(ArpSettings* settings) {
	uint16_t note_to_play;
	if (!ready_for_new_note(settings)) {
		return;
	}

	note_to_play = get_next_note(settings);
	play_note(note_to_play, settings);
	settings->notes_played_this_iter += 1;
	handle_reset_logic(settings);
}

uint16_t get_next_note(ArpSettings* settings) {
	uint16_t current_note = settings->root_note_num;
	if (settings->direction == ARP_UP) {
		// Handle overflow.
		if ((current_note + settings->interval * settings->notes_played_this_iter) >= 256) {
			if (settings->wrap_notes) {
				return (current_note + settings->interval * settings->notes_played_this_iter) % 256;
			}

			return 255;
		}

		return current_note + settings->interval;
	}

	// Handle underflow.
	if (((int16_t)current_note - (int16_t)(settings->interval * settings->notes_played_this_iter)) < 0) {
		if (settings->wrap_notes) {
			return (current_note - settings->interval * settings->notes_played_this_iter) % 256;
		}

		return 0;
	}

	return current_note - settings->interval * settings->notes_played_this_iter;
}

uint8_t ready_for_new_note(ArpSettings* settings) {
	// TODO: implement timer logic
	// Check if the appropriate number of timer ticks have elapsed.
	return 1;
}

void play_note(uint16_t note_to_play, ArpSettings* settings) {
	// TODO: implement play logic
	// Send note to DAC.
}

void handle_reset_logic(ArpSettings* settings) {
	if (settings->notes_played_this_iter >= settings->notes_to_play) {
		settings->notes_played_this_iter = 0;
	}
}
