#!/bin/sh

#aplay --device plughw:CARD=ArrayUAC10,DEV=0 /usr/share/sounds/alsa/Front_Center.wav
#aplay --device default:CARD=ArrayUAC10 /usr/share/sounds/alsa/Front_Center.wav

# Play a standard sound
echo 'Playback test'
aplay /usr/share/sounds/alsa/Front_Center.wav
