# Robotics 2016
============

This Repo is for the MICS 2016 robotics compitition

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
| Motor 1: Hitter Motor #1          | Motor 1: Left Drive #1              |
| Motor 2: Hitter Motor #2          | Motor 2: Left Drive #2              |
| Sensor 1: Multiplexor             | Sensor 1: Not Connected             |
| Sensor 2: Not Connected           | Sensor 2: Not Connected             |

| Multiplexer             | 
| ----------------------- |
| Motor 1: Right Drive #1 |
| Motor 2: Right Drive #2 |

#### Arduino Pins
|Thing          |Pin             |
|---------------|----------------|
|Serial LCD     |5 (Digital)     |
|Speaker        |6               |

#### Dianostics Mode
The robot has a basic diagnostics and troubleshooting mode which can be used to run scripted tests of motors or actions.

To enter diagnostics mode, power on the robot. Upon seeing the voltage readout message, or a
message instructing you to press go, hold the "Right arrow" button on the shield. While holding the right arrow button, press the "go" button. Release both buttons.

To exit diagnostics mode, reset the arduino or power cycle the whole bot.






