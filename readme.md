[![](https://jitpack.io/v/Hemlock5712/brain-lib.svg)](https://jitpack.io/#Hemlock5712/brain-lib)

<center>
<h1>Brain Lib</h1>

A general purpose library for <a href="https://www.firstinspires.org/robotics/frc">FRC</a> robot programming.

Provides several utilities that can be used from year to year.
</center>

## Installation

In vscode, add the following as a vendor dependency

```
https://raw.githubusercontent.com/Hemlock5712/brain-lib/main/brain-lib.json
```

## Functionality

### State Space Models

Brain Lib provides a few different state space models for making finely tuned subsystems.

#### Elevator

Base subsystem that handles most of the hard work for creating an elevator based subsystem.

#### Flywheel

Base subsystem for handling a flywheel subsystem. Gives utility methods to just set a speed (rpm)
and the robot should do it.