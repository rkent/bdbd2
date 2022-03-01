#!/usr/bin/env python3
#
#   Copyright 2022 R. Kent James <kent@caspia.com>
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
"""text-to-speech using google TTS using my API key, output throughLinux aplay app"""

import os
import subprocess
from tempfile import NamedTemporaryFile

from google.cloud import texttospeech

os.environ['GOOGLE_APPLICATION_CREDENTIALS'] = '/secrets/stalwart-bliss-270019-7159f52eb443.json'
#effect = 'wearable-class-device'
effect = 'small-bluetooth-speaker-class-device'

def loginfo(text):
    #print(text)
    pass

class GoogleTTS():
    def __init__(self, language='en-AU', voiceName='en-AU-Standard-B'):
        # Instantiates a client
        self._client = texttospeech.TextToSpeechClient()
        self._voice_params = texttospeech.VoiceSelectionParams(
            name=voiceName,
            language_code=language
        )
        self._audio_config =  texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.LINEAR16,
            volume_gain_db=-2.0,
            pitch=0.0,
            speaking_rate=1.0,
            effects_profile_id=[effect])

    def say(self, text):
        """Main entry, generate audio from text and play it"""
        loginfo('Asked Google to say {}'.format(text))
        text_input = texttospeech.SynthesisInput(text=text)

        # Perform the text-to-speech request on the text input with the selected
        # voice parameters and audio file type
        response = self._client.synthesize_speech(
            input=text_input,
            voice=self._voice_params,
            audio_config=self._audio_config
        )
        loginfo('Got audio content from Google')

        with NamedTemporaryFile("w+b", suffix=".wav") as f:
            f.write(response.audio_content)
            subprocess.call(['/usr/bin/aplay', '-q', f.name])
        loginfo('Done playing')

if __name__ == '__main__':
    # demo
    googleTTS = GoogleTTS()
    googleTTS.say('Hello my name is BDBD robot')
