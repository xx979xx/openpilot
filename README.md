# Welcome to Arne fork of openpilot

This README describes the custom features build by me (Arne Schwarck) on top of [openpilot](http://github.com/commaai/openpilot) of [comma.ai](http://comma.ai). This fork is optimized for the Toyota RAV4 Hybrid 2016 and for driving in Germany but also works with other cars and in other countries. If you would like to support the developement on this project feel free to https://www.patreon.com/arneschwarck 
- [ ] TODO describe which other cars and countries are known
[![](https://i.imgur.com/UelUjKAh.png)](#)

For a demo of this version of openpilot check the video below:
[![demo of openpilot with this branch](https://img.youtube.com/vi/WKwSq8TPdpo/0.jpg)](https://www.youtube.com/playlist?list=PL3CGUyxys8DuTE1JTkdZwY93ejSfAGxyV)

# Installation
`cd /data/openpilot; rm -rf openpilot; git clone https://github.com/arne182/openpilot; cd openpilot; git checkout release4; reboot`

## Panda flashing

To get this branch to work, it is required to flash your Panda because:
- changing acceleration limits and 
- adapting lane departure warning where it gives you a slight push back into the middle of the lane without needing to be engaged
- The Panda version is also changed and checked.
- Run this command `cd /data/openpilot/panda/board; make; reboot` to manually flash the panda.

More info about Panda flashing can be found [here](https://community.comma.ai/wiki/index.php/Panda_Flashing).

## Installating this fork

More info about how to install this fork can be found [here](https://medium.com/@jfrux/comma-eon-installing-a-fork-of-openpilot-5c2b5c134b4b).

## Branches

`release4`: this is the default branch that is most up to date with the openpilot 0.7 release branch. Normally you should use this branch.
`072-clean`: this is my default testing branch. When I finishing testing/adding new structure, I'll merge this into the `release4` branch.
`release3`: this is my old branch, that is compatible with openpilot 0.6.
`release2`: this is my old branch, that is compatible with openpilot 0.5.

# Configuration

- You can turn on or off some of the feature by editing `op_edit.py`. run the following command `cd /data/openpilto; python op_edit.py`

# Todo

- [ ] Auto Lane change from Boggyver on release2 and release3 branch.

- [ ] Traffic light detection from Littlemountainman

- [ ] Phantom: control open pilot via app like summon

- [ ] Control 3 gas profiles with sport eco and normal buttons on car

- [ ] Dynamic gas and distance profiles.

# Features

- Braking: 
    - by angle(carstate), 
    - by predicted angle in 2.5s(laneplanner), 
    - by model(commaai), 
    - acceleration measured by steering angle, 
    - by curvature (mapd), 
    - by mapped sign(stop, yield, roundabouts, bump, hump, traffic light, speed sign, road attribute)
- No disengage for gas, only longitudinal disengage for brake, tire slip or cancel
- Only disengage on main off and on brake at low speed
- Reacting Toyota tssp higher acceleration and braking limits.
- Speed sign reading 
- Stock Toyota ldw steering assist
- Cruise set speed available down to 7 kph
- Smooth longitudinal controller also at low speeds
- No disengage for seat belt remove and door opened. Practical for when stopping and then someone opens a door so that the car does not drive into the lead
- No fingerprint compatibility problems. A completely different way to combine and split Fingerprints so that they always work I.e. comma is not supporting rav4h 2019 because of this Fingerprint method. Mine is better
- Custom events and capnp structure so that comma is happy with the drives from my fork
- Forward collision warning actually brakes for you.
- Blind Spot Monitoring for all of the toyota which will be added to control ALCA. For right now it is always on. It will flash rapidly when stopped and if the object is detected. 

# Licensing

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>
