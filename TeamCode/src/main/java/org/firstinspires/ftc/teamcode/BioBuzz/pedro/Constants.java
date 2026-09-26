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

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c ->
    {
        c.frontLeftName.set("FL");
        c.frontRightName.set("FR");
        c.backLeftName.set("BL");
        c.backRightName.set("BR");//
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("ODO");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.7667248643289402);//9/26/26 Andrew
        c.yPodOffset.set(1.3585834052619032);//9/26/26 Andrew
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);//9/26/26 Andrew
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);//9/26/26 Andrew
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig( //9/26/26 Andrew
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.22365199794622623);
                Controller secondaryTranslationalForward = Controller.proportional(0.08263343714968223);
                Controller primaryTranslationalLateral = Controller.proportional(0.30061950431523);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1110708741433818);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017036527856351753));
                c.brake.set(Controller.proportionalFeedforward(0.014481048677898989));

                c.headingFeedback.set(Controller.proportional(3.2972802555476366));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04318585533766321, 0.00924851966055401));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07003350461230726, 0.03534497619135207));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0025715332770722362, 0.0030457111058454804));

                c.maxAchievableForwardVelocity.set(60.99222275371436);
                c.maxAchievableStrafeVelocity.set(51.14228474869199);
                c.naturalForwardDeceleration.set(35.100531899727315);
                c.naturalStrafeDeceleration.set(51.6276141088511);
            }
    );


    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
}
