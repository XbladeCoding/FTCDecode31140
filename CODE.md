# How the Code Works

[← Back to README](../README.md)

## Robot hardware

| Subsystem | Hardware | Config names |
|---|---|---|
| Drivetrain | 4-motor mecanum | `leftFrontDrive`, `rightFrontDrive`, `leftRearDrive`, `rightRearDrive` |
| Odometry | 3 dead wheels, plugged into the drive motors' encoder ports | (shared with drive motors) |
| Intake | CR servo + 2 conveyor motors | `intakeServo`, `leftConveyor`, `rightConveyor` |
| Launcher | 2 motors | `leftLaunch`, `rightLaunch` |

The names in the code have to match the robot's configuration on the Control Hub.

## Autonomous structure

Each autonomous is a **state machine**. A `pathState` counter steps through a sequence, and it only moves on once the robot has finished its current path or the current timer has run out:

```
Drive to scoring spot → Shoot preload → Drive through row 1 with intake on
  → Return and shoot → Row 2 → Row 3 → Park
```

- **Coordinates.** `Pose(x, y, heading)` gives positions in inches on the 144 × 144 in field. For example, red starts at `(114, 135, 90°)` and shoots from `(96, 96, 45°)`.
- **Paths.** Each leg is a `BezierLine` between two poses, and the heading changes smoothly along the way.
- **Intake on the move.** Pickup paths are split into short segments, so the robot slows down while it collects.
- **Voltage compensation.** At startup the code reads the battery voltage and sets launcher power with `shootConstant = -0.0338 × voltage + 0.781`.
- **Shot boost.** Partway through each shot, the launcher power goes up briefly so the flywheels recover speed between artifacts.

## Pedro Pathing

[Pedro Pathing](https://pedropathing.com/) follows a planned path and keeps correcting the robot's position, heading and speed with PID controllers, so the robot stays on course even if it gets bumped.

Dependencies (`build.dependencies.gradle`):
- `com.pedropathing:ftc:2.0.5`
- `com.pedropathing:telemetry:1.0.0`

`Constants.java` holds:
- the drivetrain motor names and directions
- the dead-wheel offsets and the encoder ticks-to-inches conversion
- the robot's mass (10.6 kg) and measured top speeds
- the tuned PIDF coefficients

## Tuning

Run the **`Tuning`** OpMode and work through it in this order. Copy each result into `Constants.java` as you go.

1. **Localization:** the Forward, Lateral and Turn tuners
2. **Automatic:** the velocity and zero-power acceleration tuners
3. **PIDF:** Translational, Heading, Drive and Centripetal
4. **Tests:** the Line, Triangle and Circle paths

See the [Pedro Pathing tuning guide](https://pedropathing.com/) for details.

## Building

1. Install [Android Studio](https://developer.android.com/studio) (Ladybug 2024.2 or newer).
2. Clone the repo and open it. Wait for Gradle to finish syncing.
3. Connect to the Control Hub over USB or Wi-Fi, select **TeamCode**, and press **Run ▶**.
