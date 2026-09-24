package org.firstinspires.ftc.teamcode.BioBuzz.Constructors;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class QuickRigging {
    //addMotor - returns a new DcMotor with specified params. Called from teleop. Pass in hardwaremap and driverstation name


    public static DcMotor addMotor(HardwareMap hardwareMap, String DSName, boolean Direction, boolean Stop) {
        //Adds a DcMotor, params are (Name of motor, clockwise or counter, and brake or float.)
        DcMotor obj;
        obj = hardwareMap.get(DcMotor.class, DSName);
        obj.setDirection(Direction ? DcMotor.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        obj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        obj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        obj.setZeroPowerBehavior(Stop ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        return obj;
    }
    public static DcMotor addMotor(HardwareMap hardwareMap, String DSName, boolean Direction, boolean Stop, boolean encoder) {
        //Adds a DcMotor, params are (Name of motor, clockwise or counter, and brake or float, using encoder.)
        DcMotor obj;
        obj = hardwareMap.get(DcMotor.class, DSName);
        obj.setDirection(Direction ? DcMotor.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        obj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        obj.setMode(encoder ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        obj.setZeroPowerBehavior(Stop ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
        return obj;
    }
    public static Servo addServo(HardwareMap hardwareMap, String DSName, boolean Direction) {
        //Adds a Servo, params are (Name of servo, clockwise or counter.)
        Servo obj;
        obj = hardwareMap.get(Servo.class, DSName);
        obj.setDirection(Direction ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);
        return obj;
    }
    public static Servo addServo(HardwareMap hardwareMap, String DSName, boolean Direction, double StartPos) {
        //Adds a Servo, params are (Name of servo, clockwise or counter, starting position 0-1.)
        Servo obj;
        obj = hardwareMap.get(Servo.class, DSName);
        obj.setDirection(Direction ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);
        obj.setPosition(StartPos);
        return obj;
    }


    public static CRServo addCRServo(HardwareMap hardwareMap, String DSName, boolean Direction) {
        //Adds a CRServo, params are (Name of servo, clockwise or counter, starting power -1 to 1.)
        CRServo obj;
        obj = hardwareMap.get(CRServo.class, DSName);
        obj.setDirection(Direction ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
        return obj;
    }



}

