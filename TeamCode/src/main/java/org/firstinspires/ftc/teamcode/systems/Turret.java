   package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

public class Turret {
    private LinearOpMode opMode;
    public CRServo rightTurret;
    public CRServo leftTurret;
    public AnalogInput turretAnalog ;
    private double turretOldPos;
    private double trueRotation;



    public Turret(LinearOpMode opMode) {
        this.opMode = opMode;
        rightTurret =opMode.hardwareMap.get(CRServo.class, "rightTurret");
        leftTurret = opMode.hardwareMap.get(CRServo.class, "leftTurret");
        turretAnalog = opMode.hardwareMap.get(AnalogInput.class, "turretAnalog");


    }
    public void moveTurret(double power) {
        rightTurret.setPower(power);
        leftTurret.setPower(power);
    }



    public static double getRotationOfInput(AnalogInput input) {
        return (input.getVoltage() / input.getMaxVoltage()) * 360;
    }

    public void updateTurretServoRotation() {
        double currentRotation = getRotationOfInput(turretAnalog);
        double diff = currentRotation - turretOldPos;

        double newRotationEstimate = 165;
        if(Math.abs(diff) > newRotationEstimate){
            //new rotation occur
            if(diff < 0)
                diff += 360; //add rotation
            else
                diff -= 360; //minus rotation

        }


        turretOldPos = currentRotation;
        trueRotation += diff;
        opMode.telemetry.addData("rottion", trueRotation);

    }
}



