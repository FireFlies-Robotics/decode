package org.firstinspires.ftc.teamcode.systemTeleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.intellij.lang.annotations.JdkConstants;

@TeleOp(name = "shooter Control", group = "TeleOp")
@Config
public class ShooterCheck extends LinearOpMode {
    public static double targetVel = 2000;
    Hood hood;
    Shooter shooter;
    Intake intake;
    Transfer transfer;
    public static double pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(this);
        intake = new Intake(this);
        transfer =new Transfer(this);
        hood = new Hood(this);
        shooter.setShotingPower(0);
        hood.setPosition(pos);
        waitForStart();
        while (opModeIsActive()){
            shooter.shooterPID(targetVel);
            hood.setPosition(pos);
            if (gamepad1.a) {
                intake.activateIntake(1);
                transfer.setTransferPower(1);
            }
            else
            {
                intake.activateIntake(0);
                transfer.setTransferPower(0);
            }
        }
    }
}