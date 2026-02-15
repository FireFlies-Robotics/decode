package org.firstinspires.ftc.teamcode.automations;

import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Wheels;

import java.util.logging.Handler;

public class ShootWithDistance {
    Shooter shooter;
    Hood hood;
    Wheels wheels;

    public static double flywheelSpeed(double goalDistance){
        return -0.0000673324 * Math.pow(goalDistance, 3)
                + 0.0259075 * Math.pow(goalDistance, 2)
                - 0.503649 * goalDistance
                + 987.69667;
    }
    public static double hoodAngle(double goalDistance){
        return -0.000544092 * goalDistance + 0.415277;
    }
    public void changeToShoot(){
        double distance = wheels.getDistanceFromGoal();

        shooter.shooterPID(flywheelSpeed(distance));
        hood.setPosition(hoodAngle(distance));

    }
}
