# SLUcontroller

slu_main should be run as `python3 slu_main.py --slu /dev/ttyUSB` (or other path which corresponds to serial cable of slu).

`debug.py` should be copied to `FARBOT/webrtc/py/` in the jetson nano.

The executable file should be created by `pyinstaller debug.py --onefile` at `FARBOT/webrtc/py/`. New executable file will be created at `FARBOT/webrtc/py/dist/debug`, which should be copied to `FARBOT/webrtc/py/`.
The new executable file for the current code is there in executablefiles of this repo which can be copied.

The IP adress and controller should be changed in the config file of atdrive-moab and should be compiled into moab board. The new config file is there in the repository `ROBOT_CONFIG.hpp`. The new binary file `slucopy.bin` is also there in the repository, which can be copied to the board.
