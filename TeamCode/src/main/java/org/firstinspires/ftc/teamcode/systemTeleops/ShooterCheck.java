package org.firstinspires.ftc.teamcode.systemTeleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Shooter;
@TeleOp(name = "shooter Control", group = "TeleOp")

public class ShooterCheck extends LinearOpMode {

    Shooter shooter;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(this);
        shooter.setShotingPower(0);
        waitForStart();
        while (opModeIsActive()){
            shooter.shooterPID(900);

        }
    }
}