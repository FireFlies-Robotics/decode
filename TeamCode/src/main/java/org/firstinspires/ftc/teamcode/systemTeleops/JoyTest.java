package org.firstinspires.ftc.teamcode.systemTeleops;

import android.provider.ContactsContract;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Hood;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Transfer;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.systems.Wheels;

@Autonomous(name = "jotstick test")
//@Disabled
public class JoyTest extends LinearOpMode {

    Wheels wheels;
    IMU imu; // Declare class for getting robot angles
    Transfer transfer;
    Shooter shooter;
    Intake intake;
    Turret turret;
    private ElapsedTime runtime = new ElapsedTime();


    Hood hood;


    @Override
    public void runOpMode() {
        telemetry.addLine("Initialized — Ready to start");
        telemetry.update();
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)));
        imu.resetYaw();
        turret = new Turret(this, imu);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("angle", Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)) + 180);
            telemetry.addData("calculated angle", turret.calculateTargetRotation());
            telemetry.addData("imu", imu.getRobotYawPitchRollAngles().getYaw());

            turret.setTurretPosition();
            turret.updateTurretServoRotation();
            telemetry.update();
        }
    }
}
