#!/bin/sh

#aplay --device plughw:CARD=ArrayUAC10,DEV=0 /usr/share/sounds/alsa/Front_Center.wav
#aplay --device default:CARD=ArrayUAC10 /usr/share/sounds/alsa/Front_Center.wav

# Play a standard sound
echo 'Playback test'
aplay /usr/share/sounds/alsa/Front_Center.wav

echo 'Talk for 5 seconds'
arecord -d 5 -f s32_LE -r 16000 /tmp/test.wav

echo 'Playing back what you said'
aplay /tmp/test.wav
