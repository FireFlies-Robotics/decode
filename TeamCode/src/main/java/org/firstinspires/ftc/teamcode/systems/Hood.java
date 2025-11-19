package org.firstinspires.ftc.teamcode.systems;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood {

    public Hood(LinearOpMode opMode){
        hoodServo = opMode.hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(0);
    }
    private Servo hoodServo;
//    private double ServoPosition;
    public void setPosition(double position){
        hoodServo.setPosition(position);
    }

    public void changeAngle(){


//        InterpLUT lut = new InterpLUT();
//        lut.add(1100, 0.2);
//        lut.add(2.7, .5);
//        lut.add(3.6, 0.75);
//        lut.add(4.1, 0.9);
//        lut.add(5, 1);
//
//        lut.createLUT();


        double distance = 1500;
//        hoodServo.setPosition(lut.get(distance));
    }
}
