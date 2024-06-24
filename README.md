## Team 6512 (Functioning) Swerve Code
This is the swerve code we will use as a foundation to build off of for our 2024-25 robot, which will be our first robot with a swerve drivetrain. 
We are using the SDS Mk4 Swerve Modules with REV Neos and Spark MAX Motor controllers. For encoders, we are using Thrifty absolute magnetic encoders and the built-in Spark Max relative encoders.

### Current Status
- Code Functions Properly(ish)
- Turning sensitive

### To Do
- Have programming team analyze code to make sure all members fully understand everything and where we are starting from.
- Work on more advanced stuff like kinematics / odometry for smoother driving.
- Work on autonomous using PathPlanner / PathWeaver
- Test robot and field orientation after adding gyro

### Created with help from these projects:
- - [4534's 2023 Swerve Code](https://github.com/4534-WiredWizards/4534-Swerve-Public) - Thanks to our mentor team, the Wired Wizards, for helping us get our swerve drivetrain up and working, along with providing their swerve code.
- - [2495's Edited MAXSwerve Template](https://github.com/FRC2495/MAXSwerve) - Used ThriftyEncoder class from this project, as we couldn't get the Thrifty absolute magnetic encoders working with base WPILIB libraries.
  - [6624's Swerve Example](https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html) - Although this code caused our drivetrain to behave strangely, 6624's swerve documentation gave us a good idea of how many of the concepts that swerve is based around worked.
