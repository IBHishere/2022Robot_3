
## Subsystems

  ### Subsystem1 Name
  Subsystem description
  
  ### Commands
  List and describe commands
  
## Instructions

### Useful pages
   WPILib Java Examples: https://docs.wpilib.org/en/stable/docs/software/examples-tutorials/wpilib-examples.html
   

### How to add a subsytem and commands
   1. Define a subsystem class
         1. Define components of the subsystem in the subsystem class - motors, motor groups, sensors
         1. Define functions - at a basic level what can this subsystem do - shoot, drive, etc.
         1. Publish telementry through NetworkTables
   1. Define command classes
         1. There can be more than one - collect balls, eject balls
         2. Pass subsystem to the constructor
   1. Wire up subsystems, commands and buttons in RobotContainer
         1. Instance of subsystem
         2. Wireup code

### Git commands    
```git clone https://github.com/FRCTeam1719/2022Robot_3``` commeand to get brand new repository

```git pull``` pull down any updates from github

```git add . ``` add all files that were added or modified to the local repo

```git commit -m "<message goes here>" ``` stage files to commit

```git push``` push your commits to github

## Programmers:

Matthew Wetzler @mwetz08(Lead Programmer)
------------------------------------------------------
Sam Rabb-Jaros  @GethrexFe(Head assistant Programmer
