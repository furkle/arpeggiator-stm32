/*
 * arp.h
 *
 *  Created on: Sep 2, 2021
 *      Author: furkle
 */

#ifndef INC_ARP_H_
#define INC_ARP_H_


typedef enum ArpDirection {
	ARP_DOWN,
	ARP_UP
} ArpDirection;

typedef enum PitchInterval {
	MINOR_SECOND = 1,
	MAJOR_SECOND,
	MINOR_THIRD,
	MAJOR_THIRD,
	PERFECT_FOURTH,
	TRITONE,
	PERFECT_FIFTH,
	MINOR_SIXTH,
	MAJOR_SIXTH,
	MINOR_SEVENTH,
	MAJOR_SEVENTH,
	PERFECT_OCTAVE
} PitchInterval;

typedef enum BeatDivision {
	THIRTY_SECOND_BEAT = 1,
	SIXTEENTH_BEAT = 2,
	THREE_THIRTY_SECONDTHS_BEAT = 3,
	EIGHTH_BEAT = 4,
	THREE_SIXTEENTHS_BEAT = 6,
	QUARTER_BEAT = 8,
	THREE_EIGHTHS_BEAT = 12,
	HALF_BEAT = 16,
	FIVE_EIGHTHS_BEAT = 20,
	FULL_BEAT = 32
} BeatDivision;

typedef struct ArpSettings {
	volatile uint16_t bpm;
	volatile PitchInterval interval;
	volatile BeatDivision beat_division;
	volatile uint8_t notes_to_play;
	volatile uint8_t direction;
	volatile uint32_t last_seen_time;
	volatile uint8_t notes_played_this_iter;
	volatile uint8_t root_note_num;
	volatile uint8_t wrap_notes;
} ArpSettings;

void arp_init(ArpSettings* settings);
void bpm_init(ArpSettings* settings);
void update_bpm(ArpSettings* settings);
void arpeggiate(ArpSettings* settings);
uint16_t get_next_note(ArpSettings* settings);
uint8_t ready_for_new_note(ArpSettings* settings);
void play_note(uint16_t note_to_play, ArpSettings* settings);
void handle_reset_logic(ArpSettings* settings);

#endif /* INC_ARP_H_ */
