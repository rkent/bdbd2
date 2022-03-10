#/bin/bash
# Run this once to prep jetbot to work with bdbd2
cat config/bashrc_add.sh >> ~/.bashrc

# Caution - this seems to overwrite a 'ln -s' file rather than replace the link.
sudo cp config/asound.conf /etc/
sudo cp config/60-respeaker.rules /etc/udev/rules.d/
