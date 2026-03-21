# Plan: Add Intake to AutoAimAndShoot

## Overview
Add the Intake subsystem to `AutoAimAndShoot` so that the intake runs alongside the spindexer and feeder during the feeding phase.

## Changes Required

### 1. Modify AutoAimAndShoot.java
**File:** [AutoAimAndShoot.java](src/main/java/frc/robot/commands/ShootFeedCommands/AutoScoringCommands/AutoAimAndShoot.java)

- Add `Intake` import
- Add `m_intake` member variable
- Update constructor to accept `Intake` parameter
- Add `intake` to `addRequirements()`
- In `updateFeedState()`: call `m_intake.setSpeed(MotorConstants.kIntakeRollerSpeed)` when feeding
- In `updateFeedState()`: call `m_intake.stopIntakeMotor()` when not feeding
- In `execute()` no-target branch: call `m_intake.stopIntakeMotor()`
- In `end()`: call `m_intake.stopIntakeMotor()`

### 2. Update Bindings.java
**File:** [Bindings.java](src/main/java/frc/robot/Bindings.java)

- Line 82: Add `intake` parameter to `AutoAimAndShoot` constructor call

### 3. Update Autos.java
**File:** [Autos.java](src/main/java/frc/robot/Autos.java)

- Line 122: Add `intake` parameter to first `AutoAimAndShoot` instance
- Line 139: Add `intake` parameter to second `AutoAimAndShoot` instance
- Update `Set.of()` calls on lines 134 and 151 to include `intake`

---

## Regarding Your Question: Multiple Constructors

Yes, having two constructors with different subsystem parameters is valid Java and works with WPILib's command framework. However, there are trade-offs:

**Pros:**
- Backwards compatibility if existing code doesn't need intake
- Flexibility for different use cases

**Cons:**
- The `addRequirements()` call must differ between constructors - if one constructor doesn't include `intake`, it won't require that subsystem, which could cause scheduling conflicts if another command tries to use intake simultaneously
- More code to maintain
- Can lead to subtle bugs if the command behavior differs based on whether intake is provided

**Recommendation:** For this case, a single constructor is cleaner since:
1. There are only 3 call sites to update
2. The intake should always run during auto-aim shooting (that's the desired behavior)
3. It avoids potential subsystem requirement conflicts

If you wanted backwards compatibility, you could do:
```java
public AutoAimAndShoot(Swerve swerve, Flywheel flywheel, TopRoller topRoller,
                       Feeder feeder, Spindexer spindexer,
                       DoubleSupplier translationX, DoubleSupplier translationY) {
    this(swerve, flywheel, topRoller, feeder, spindexer, null, translationX, translationY);
}

public AutoAimAndShoot(Swerve swerve, Flywheel flywheel, TopRoller topRoller,
                       Feeder feeder, Spindexer spindexer, Intake intake,
                       DoubleSupplier translationX, DoubleSupplier translationY) {
    // ... implementation with null checks for intake
}
```

But this adds complexity for no real benefit here.
