package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Turret {
    private LinearOpMode opMode;
    public CRServo rightTurret;
    public CRServo leftTurret;

    public Turret(LinearOpMode opMode) {
        this.opMode = opMode;
        rightTurret =opMode.hardwareMap.get(CRServo.class, "rightTurret");
        leftTurret = opMode.hardwareMap.get(CRServo.class, "leftTurret");
    }
    public void moveTurret(double power) {
        rightTurret.setPower(power);
        leftTurret.setPower(power);


    }
}
