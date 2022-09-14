#!/bin/bash

FILES="/data/bitmap/NaoSpaceInvaders_Summer2022/game_bags/*"

for f in $FILES
do
    echo "Processing $f file..."
    base_name=$(basename ${f})
    rosrun img_audio_convert bag2wav -b $f --output=/data/bitmap/NaoSpaceInvaders_Summer2022/game_wavs/${base_name%.bag}.wav --input-audio-topic=/audio/audio
done