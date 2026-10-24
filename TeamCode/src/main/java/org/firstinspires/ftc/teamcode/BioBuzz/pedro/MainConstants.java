package org.firstinspires.ftc.teamcode.BioBuzz.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MainConstants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("ODO");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(5.829839030588706);
        c.yPodOffset.set(-1.5853725643608516);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3012016224062592);
                Controller secondaryTranslationalForward = Controller.proportional(0.11128595122353525);
                Controller primaryTranslationalLateral = Controller.proportional(0.7187197168511823);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2655477308319824);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.027442626012514355));
                c.brake.set(Controller.proportionalFeedforward(0.023326232110637203));

                c.headingFeedback.set(Controller.proportional(2.5190073793279217));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04944737945816706, 0.0037507366505340973));

                c.linearBrakeCoefficients.set(Matrix.diag(0.054079647954859596, 0.06662825265709749));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0018074730343767965, 0.0012720283655267145));

                c.maxAchievableForwardVelocity.set(41.30209440198968);
                c.maxAchievableStrafeVelocity.set(42.25813047684851);
                c.naturalForwardDeceleration.set(59.06528695258176);
                c.naturalStrafeDeceleration.set(87.21180135313676);
            }
    );


    public static Follower createFollower(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
}
