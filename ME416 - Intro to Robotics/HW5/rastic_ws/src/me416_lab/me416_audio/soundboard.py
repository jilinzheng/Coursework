'''
Sound FX
'''

import os

from pygame.mixer import Sound

FILE_PATH = os.path.dirname(os.path.realpath(__file__))

names = [s.replace('.wav', '') for s in os.listdir(FILE_PATH) if '.wav' in s]

sounds = {
    name: Sound(os.path.join(FILE_PATH, name + '.wav'))
    for name in names
}


def play(name):
    '''
    Play a sound effect
    '''
    sounds[name].play()
