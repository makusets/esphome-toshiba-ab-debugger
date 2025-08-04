# Ordering or building the board

You will need to build the compatible hardware. This folder contains all necessary files. I designed the board using EasyEDA online editor (free). All files are EasyEDA compatible format.
EasyEDA.zip file can be imported onto EasyEDA editor to open the project with schematics and board design.

Alternatively, you can order the board from places like JLCPCB. You will need the 3 files included:

  - Gerber.zip: This is the gerber file for the physical design of the board
  - BOM_01.csv or BOM_02.csv: Just choose one of them, they are slightly different format to maximise compatibility. They contain the list of components (BOM).
  - PickAndPlace.csv: The information on where to place the component on the board.

## Component shortfall

It often happens that components are out of stock or become unavailable and need to be replaced when uploading the BOM file. If you are not comfortable in doing so,
tick the option of "parts sourced by JLCPCB" instead of "by customer". That should make it easy to replace shortfalls.


![image](hardware/Board.JPG)

# Schematic

This is the schematic of the board.

![image](hardware/Schematic.JPG)

