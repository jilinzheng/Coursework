'''
Wraps pygame to add utility classes and functions
'''

from pygame import mixer, time
from pygame.mixer import music

if not mixer.get_init():
    try:
        mixer.init(size=32)
    except ValueError:
        # Change to 16-bit audio if 32-bit is not supported
        mixer.init(size=16)

from . import notes, soundboard


def test_notes():
    '''
    Test all notes
    '''
    duration = 0.5
    octave = -1
    # Play chromatic scale
    for note in notes.names:
        notes.play(note, duration, octave)
        time.wait(500)


def test_music():
    '''
    Test playing background music
    '''
    duration = 5  # seconds
    music.load('blue_danube.ogg')
    music.play()
    time.wait(duration * 1000)
    music.stop()


def test_soundboard():
    '''
    Test all sound effects
    '''
    for name in soundboard.names:
        soundboard.play(name)
        time.wait(500)


if __name__ == '__main__':
    test_notes()
    # test_ogg()
