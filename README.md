ESP-TNC

Supports Ai Thinker Audio Dev Board with AC101 codec; minor refactoring and enhancement could support Espressif Lyra board.

This is an ESP-IDF v4.0 project. To get started, clone, cd into clone, do 'git submodule update --init', then 'idf.py build'

Notes:

Decode performance is best with audio input level 'hot' on received packets from discriminator output (not de-emphasized audio) (in open-squelch operation, ignore non-packet noise level, it will clip, that's fine). Some clipping on received packets is to be expected.

PTT is IO_23, active high. Sorry about hiding that in the codec initialization.

