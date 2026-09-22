package org.firstinspires.ftc.teamcode.BioBuzz.Constructors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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



}

