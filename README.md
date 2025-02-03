# BunnyACE

A Work-In-Progress driver for Anycubic Color Engine Pro for Klipper

## Installation
The module can be installed into a existing Klipper installation with an install script. 

    cd ~
    git clone https://github.com/BlackFrogKok/BunnyACE.git
    cd BunnyACE
    ./install.sh

## Uninstall

Remove all BunnyAce definitions in your Klipper configuration and the updater
section in the Moonraker configuration. Then run the script to remove the link:

    cd ~
    cd BunnyACE
    ./install.sh -u

## Pinout

![Molex](/.github/img/molex.png)

- 1 - None (VCC, not required to work, ACE provides it's own power)
- 2 - Ground
- 3 - D-
- 4 - D+

Connect them to a regular USB, no dark magic is required.