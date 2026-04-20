/*
 * buzzer.h
 *
 *  Created on: Mar 31, 2026
 *      Author: Sigmond
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

typedef struct {
	uint16_t freq;   // [Hz] or 0 for off
	uint16_t duration; // [ms]
} buzzer_beep_t;

typedef struct {
	TIM_HandleTypeDef* handle;
	uint32_t tim_freq;

	// music player
	const buzzer_beep_t* seq;
	uint16_t seq_len;
	uint16_t seq_idx;
	uint32_t beep_start; // [ms] since boot (hal get tick)
	uint8_t seq_playing;
} buzzer_t;

void buzzer_set(buzzer_t* buzzer, uint8_t status) {
	uint32_t ccr;

	if (status == 0) {
		ccr = 0;
	}

	if (status == 1) {
		ccr = 8000;
	}

	// update duty cycles
	__HAL_TIM_SET_COMPARE(buzzer->handle, TIM_CHANNEL_1, ccr);
}

void buzzer_init(buzzer_t* buzzer) {
	buzzer_set(buzzer, 0); // turn channel off

	// start PWM
	HAL_TIM_PWM_Start(buzzer->handle, TIM_CHANNEL_1);
}

void buzzer_play_tone(buzzer_t* buzzer, uint16_t freq) {
	if (freq == 0) { // silent
		__HAL_TIM_SET_COMPARE(buzzer->handle, TIM_CHANNEL_1, 0); // don't play
	} else { // sound playing
		uint32_t arr = (buzzer->tim_freq / freq) - 1; // calculate period for given frequency
		__HAL_TIM_SET_AUTORELOAD(buzzer->handle, arr); // put period in timer auto reload register

		__HAL_TIM_SET_COMPARE(buzzer->handle, TIM_CHANNEL_1, arr / 2); // update the duty cycle so it stays at 50%
		__HAL_TIM_SET_COUNTER(buzzer->handle, 0); // reset to apply it right away
	}
}

void buzzer_play_sequence(buzzer_t* buzzer, const buzzer_beep_t* seq, uint16_t len) {
	buzzer->seq = seq;
	buzzer->seq_len = len;
	buzzer->seq_idx = 0;
	buzzer->seq_playing = 1;
	buzzer->beep_start = HAL_GetTick(); // record start time

	buzzer_play_tone(buzzer, seq[0].freq); // first beep in seq
}

void buzzer_update(buzzer_t* buzzer) {
	if (!buzzer->seq_playing) { // skip if not playing anything
		return;
	}

	uint32_t now = HAL_GetTick();
	uint16_t duration = buzzer->seq[buzzer->seq_idx].duration;

	if ((now - buzzer->beep_start) >= duration) { // if time since start is >= duration
		buzzer->seq_idx++; // move to next note

		if (buzzer->seq_idx >= buzzer->seq_len) { // if sequence done
			buzzer->seq_playing = 0; // stop playing
			buzzer_play_tone(buzzer, 0); // turn it off
		} else {
			// move on to next note
			buzzer_play_tone(buzzer, buzzer->seq[buzzer->seq_idx].freq);
			buzzer->beep_start = now;
		}
	}
}

#endif /* INC_BUZZER_H_ */
