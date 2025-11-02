package org.firstinspires.ftc.teamcode.systemTeleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.Shooter;

public class ShooterCheck extends LinearOpMode {
    Shooter shooter = new Shooter(this);
    @Override
    public void runOpMode() throws InterruptedException {
        shooter.setShotingPower(0);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad2.left_bumper){
                shooter.setShotingPower(1);
            }
            else {
                shooter.setShotingPower(0);
            }
        }
    }

}
