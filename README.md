~~Robotics 2014~~ Robotics 2015
============

This Repo is for the MICS ~~2014~~ 2015 Robotics compitition

This year our team tried to use an arduino with the NXShield created by OpenElectionics.
A user guide can be found here: http://www.openelectrons.com/index.php?module=documents&JAS_DocumentManager_op=viewDocument&JAS_Document_id=1, as well as an advanced guide and a tutorial.
The user guide contains a lot of useful links, but the key ones can be found below.

## Links

Arduino Software download: http://arduino.cc/en/Main/Software#.UwpVRrVDuP8

NXShield Library download: http://sourceforge.net/projects/nxshield/files/. _Note:_ The NXShield_examples folder contains useful examples.

Library API: http://www.openelectrons.com/NXShield/html/

## Pin Mappings and Hardware Configuration

> Much knowledge about pin mappings has been lost due to people not pushing to github, but it shouldn't be
too hard to figure it all out again.

#### NX-Shield NXT Ports
|Bank A                             | Bank B                              |
| --------------------------------- | ----------------------------------- |
| Motor 1: Mal (back left)          | Motor 1: Jayne (front left)         |
| Motor 2: Simon (back right)       | Motor 2: Wash (front right)         |
| Sensor 1: Multiplexor             | Sensor 1: River (US, right)         |
| Sensor 2: Lawrence (US, left)     | Sensor 2: Fanty (IR, rear facing)   |

| Multiplexor            | 
| ---------------------- |
| Motor 1: Inara (chain) |
| Motor 2: Mal (chain)   |

#### Arduino Pins
|Thing          |Pin             |
|---------------|----------------|
|Serial LCD     |~~9~~ 8 (Digital)     |
|H-Bridge pin 1 |13              |
|H-Bridge pin 2 |11              |
|H-Bridge pin 3 |10              |
|H-Bridge pin 4 |9               |
|Speaker        |6               |
|Front Rollers  |5               |



