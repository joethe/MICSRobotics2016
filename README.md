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

| Multiplexer            | 
| ---------------------- |
| Motor 1: Inara (chain) |
| Motor 2: Mal (chain)   |

#### Arduino Pins
|Thing          |Pin             |
|---------------|----------------|
|Serial LCD     |~~9~~ ~~8~~ 5 (Digital)     |
|Speaker        |6               |

#### Dianostics Mode
The robot has a basic diagnostics and troubleshooting mode which constantly prints out all sensor
values to the LCD, as well as allowing manual control of the chain motors.

To enter diagnostics mode, power on the robot. Upon seeing the voltage readout message, or a
message instructing you to press go, hold the "Right arrow" button on the shield. While holding the right arrow button, press the "go" button. Release both buttons.

To exit diagnostics mode, reset the arduino or power cycle the whole bot.

Controls within diagnostics mode:

| Button | Function |
| ------ | -------- |
| Right arrow | Lower the chain |
| Left arrow | Raise the chain |





