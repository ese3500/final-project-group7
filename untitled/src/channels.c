//
// Created by Jun Kim on 4/7/23.
//

#include "sequencer.h"

// initiate midi sequence with all zeroes
void initTrack(track t) {
    for (int i =0; i < SEQUENCE_LENGTH_MAX; i++) {
        t.sequence[i] = 0;
    }
}

void play() {
    MIDI.sendNoteOn()
}