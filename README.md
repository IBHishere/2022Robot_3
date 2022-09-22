## CAN IDs

| Motor Label   | Motor Location | CAN ID |
| ------------- | -------------- | ------ |
| Motor #1      | Left Drive     | 1      |
| Motor #2      | Left Drive     | 2      |
| Motor #3      | Right Drive    | 3      |
| Motor #4      | Right Drive    | 4      |
| Motor #5      | Intake         | 5      |
| Motor #6      | Shooter        | 6      |
| Motor #7      | Queue Loading  | 7      |
| Motor #8      | Queue Shooter  | 8      |

## Button Mappings:
### Driver Controller: 
      Left Axis: Controls left motors /n
      Right Axis: Controls right motors /n
### Helper Controller:
      Left Stick: Push out balls from intake /n
      Right Stick: Pull in balls into intake /n
      A: Start shoot queue /n
      B: Stop shoot queue /n
      X: Start shooting /n
      Y: Stop shooting /n


## Instructions

### Useful pages
   Setup: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/offline-installation-preparations.html
   
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

// Matthew Wetzler @mwetz08(Lead Programmer)
------------------------------------------------------
Sam Rabb-Jaros  @GethrexFe(Lead programmer 1)
______________________________________________________
Jai Setty @IBHishere(Lead Programmer 2)
