ESP-TNC

Supports Ai Thinker Audio Dev Board with AC101 codec; minor refactoring and enhancement could support Espressif Lyra board.

This is an ESP-IDF v4.0 project. To get started, clone, cd into clone, do 'git submodule update --init', then 'idf.py build'

Notes:

Audio in/out is on the right channel (I'd started with left, but then Espressif fixed a bug that swapped the I2S channels and I didn't care to re-wire my prototype).

Update these lines in app_main.c for your WiFi configuration:

#define DEFAULT_SSID    "SSID"
#define DEFAULT_PWD     "PASSWORD"

TCP KISS is at port 8001

Decode performance is best with audio input level 'hot' on received packets from discriminator output (not de-emphasized audio) (in open-squelch operation, ignore non-packet noise level, it will clip, that's fine). Some clipping on received packets is to be expected.

PTT is IO_23, active high. Sorry about hiding that in the codec initialization.

IO_22 is high during receive modem processing for measurement. There's a bit of a pipeline of audio samples from the ESP-IDF DMA code.


