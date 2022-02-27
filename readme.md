# MecanumPathfinderFRC
Basic implementation of Pathfinder using a 2022 FRC project. The only thing
that's been added to this project are the files in the `venderdeps/` directory.
All code is on [GitHub](https://github.com/Wobblyyyy/MecanumPathfinderFRC).

## Goals
This project is designed to demonstrate the process of creating a FRC project
that makes use of Pathfinder.

### Teleop
- Use two joysticks to control the robot
- Bind buttons to imaginary actions: (using Pathfinder's listener system)
  - A button to shoot a ball
  - A button to intake a ball

### Autonomous
- Go to a specific position (position 1)
- Perform an action (say, shooting a ball)
- Go to another position (position 2)
- Perform another action (maybe intaking another ball?)
- Return to position 1
- Shoot another ball
- Return to the starting position
