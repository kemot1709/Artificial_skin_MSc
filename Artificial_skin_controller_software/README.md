# "Lasciate ogni speranza, voi ch’entrate"

I do not recommend compiling this code on your own. Prepare to spend at least one week to set up the environment.

## HEX file upload

Prepare:
- ST Link v2 programmer (can be cheap version)
- STM32 ST-LINK Utility (https://www.st.com/en/development-tools/stsw-link004.html)
- precompiled hex file (available in this repository)

Upload:
- Connect programmed board to the programmer (check pinout on electronic schematic to avoid short circuits)
- Plug in programmer into computer (possible some driver updates or download from ST website)
- Open ST-LINK Utility and connect to the programmed board
- Open hex file through ST-LINK Utility
- Upload hex to board

## Code compilation

1. Do not compile
2. Under any circumstance
3. Unless you're a madman

Some tips:
- You will need STM32CubeMX to open _*.ioc_ file. 
- Then you should generate files with it, delete _Core_ directory and download this directory from repo again.
- Now you should have _Drivers_ dir, _*.ld_ file and few other files.
- Now you have to select IDE to compile this shit and follow its instructions about STM code compiling. Some are easier, some harder and I will not go further into this topic.
