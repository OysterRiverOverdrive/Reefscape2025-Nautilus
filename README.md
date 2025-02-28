# Overdrive - Nautilus 2025 Reefscape

Nautilus is our competition robot and code for the FIRST Robotics 2025 Competition, Reefscape. Nautilus is a modified REV MAXSwerve relying on a Studica NavX Micro as our accelerometer. Autonomous library is developed in-house.

## Contributing SOPs
 1. Changes are made on designated branches (Not Main).
 2. Any mainstream code changes require a pull request and a review from someone not on the project to merge.
 3. Any constant variables in commands or subsystems **MUST** be put in `src/main/java/frc/robot/constants.java` to be easily centralized incase of adjustment.
 4. Try to maintain general code structure (keep subsystems in the subsystems folder)

------------

## Building and Verifying Code
In order for the code to work on the robot it needs to use gradle to build then deploy
### Build
In VSCode, find the WPILib logo, in the menu type `build` and click on the menu option called `WPILib: Build Robot Code`

In the terminal, type `./gradlew build`
### Spotless
Spotless is a service that runs inorder to maintain code quality. If spotless fails when building the code. Open the terminal (Ctrl + ~) and type `./gradlew spotlessApply`

### Refreshing Dependencies
In order to deal with new dependencies, run `./gradlew build --refresh-dependencies`. This command will take longer than a normal build and needs an internet connection
