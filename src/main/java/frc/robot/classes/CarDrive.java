// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.classes;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Base driving class, takes in joysticks and performs all processing, outputting chassis speeds
 * that can then be fed into further processing
 */
public class CarDrive {
  public GenericHID controller;
  public Structs.MotionLimits motionLimits;
  public Structs.RateLimits rateLimit;

  private final DoubleSupplier steer;
  private final DoubleSupplier throttle;
  private final DoubleSupplier brake;
  private final DoubleSupplier handbrake;
  private final Supplier<Integer>[] gears = new Supplier[7];

  private final double gearMultipliers[] = {0.166, 0.333, 0.5, 0.666, 0.833, 1, -0.333};
  private final double maxAcc = 1;
  private final double maxBrake = 3;
  private final double maxSpeed = 3;
  private final double friction = 0.2;
  private final double maxSteeringRadius = 1;

  private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;

  private double xVel;

  public CarDrive(
      GenericHID controller, Structs.MotionLimits motionLimits, Structs.RateLimits rateLimits) {
    this.controller = controller;
    this.motionLimits = motionLimits;
    this.rateLimit = rateLimits;
    this.steer = () -> controller.getRawAxis(0);
    this.throttle = () -> (1 - controller.getRawAxis(2)) / 2;
    this.brake = () -> (1 - controller.getRawAxis(5)) / 2;
    this.handbrake = () -> (1 - controller.getRawAxis(1)) / 2;
    this.gears[0] = () -> controller.getRawButton(12) ? 1 : 0;
    this.gears[1] = () -> controller.getRawButton(13) ? 1 : 0;
    this.gears[2] = () -> controller.getRawButton(14) ? 1 : 0;
    this.gears[3] = () -> controller.getRawButton(15) ? 1 : 0;
    this.gears[4] = () -> controller.getRawButton(16) ? 1 : 0;
    this.gears[5] = () -> controller.getRawButton(17) ? 1 : 0;
    this.gears[6] = () -> controller.getRawButton(18) ? 1 : 0;

    xLimiter = new SlewRateLimiter(rateLimits.translationRateLimit);
    yLimiter = new SlewRateLimiter(rateLimits.translationRateLimit);
    thetaLimiter = new SlewRateLimiter(rateLimits.rotationRateLimit);
  }

  public ChassisSpeeds calculateChassisSpeeds() {

    // Drive speed
    double gearMultiplier =
        (gears[0].get() * gearMultipliers[0])
            + (gears[1].get() * gearMultipliers[1])
            + (gears[2].get() * gearMultipliers[2])
            + (gears[3].get() * gearMultipliers[3])
            + (gears[4].get() * gearMultipliers[4])
            + (gears[5].get() * gearMultipliers[5])
            + (gears[6].get() * gearMultipliers[6]);

    double acc = maxAcc * gearMultiplier * throttle.getAsDouble();
    acc -= maxBrake * brake.getAsDouble();
    acc -= friction;

    xVel += acc;

    // Steering
    double steerValue = steer.getAsDouble();
    double steerVel = 0;
    if (steerValue != 0) steerVel = xVel / (steerValue * maxSteeringRadius);

    return new ChassisSpeeds(xVel, 0, steerVel);
  }
}
