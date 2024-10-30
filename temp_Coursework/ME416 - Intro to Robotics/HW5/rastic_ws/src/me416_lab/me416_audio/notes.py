'''
Play notes using PyGame
'''

import numpy as np
from pygame import mixer

RATE = 44100

# Build 12-tone Equal Temperament chromatic scale from A to A
# Middle A is at 440Hz, 12 tones are equally spaced on a log scale
pitches = np.exp(np.linspace(np.log(220), np.log(440), 13))
# Name of notes (using sharps)
names = ['A', 'A#', 'B', 'C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A']
assert len(names) == 13  # Check that we did not forget anything
notes = dict(zip(names, pitches))


def sine(frequency, duration=1):
    '''
    Non-blocking play of a sine wave with given frequency and duration
    '''
    buffer = np.sin(2 * np.pi * np.arange(int(duration * RATE)) * frequency /
                    RATE).astype(np.float32)
    sound = mixer.Sound(buffer)
    sound.play(0)


def play(name, duration=1, octave=0):
    '''
    Play a note (named as a string in ['C','D'.'E','F','G','A','B'])
    for a given duration (in seconds). The argument octave is an integer
    that shifts the octave (negative numbers go lower, positive numbers go
    higher)
    '''
    frequency = notes[name.upper()]
    sine(frequency * (2**octave), duration)
