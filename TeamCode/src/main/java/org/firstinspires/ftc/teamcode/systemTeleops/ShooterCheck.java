package org.firstinspires.ftc.teamcode.systemTeleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.intellij.lang.annotations.JdkConstants;

@TeleOp(name = "shooter Control", group = "TeleOp")
@Config
@Disabled


public class ShooterCheck extends LinearOpMode {
    public static double targetVel = 2000;

    Shooter shooter;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(this);
        shooter.setShotingPower(0);
        waitForStart();
        while (opModeIsActive()){
            shooter.shooterPID(targetVel);

        }
    }
}