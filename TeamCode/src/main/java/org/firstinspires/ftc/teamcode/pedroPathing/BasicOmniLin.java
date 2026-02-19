package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BasicOmniLin", group = "Testing")
public class BasicOmniLin extends OpMode {
    public static class Paths {
        public PathChain Shoot1;
        public PathChain ArtifactSetup1;
        public PathChain ArtifactPickup1;
        public PathChain TravelToShoot1;
        public PathChain Shoot2;
        public PathChain ArtifactSetup2;
        public PathChain ArtifactPickup2;
        public PathChain TravelToShoot2;
        public PathChain Shoot3;
        public PathChain ArtifactSetup3;
        public PathChain ArtifactPickup3;
        public PathChain TravelToShoot3;
        public PathChain Shoot4;
        public PathChain Park;

        public Paths(Follower follower) {
            Shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(48.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                    .build();

            ArtifactSetup1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(42.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                    .build();

            ArtifactPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(42.000, 85.000), new Pose(25.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            TravelToShoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(25.000, 85.000), new Pose(48.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(48.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                    .build();

            ArtifactSetup2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(42.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                    .build();

            ArtifactPickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(42.000, 60.000), new Pose(25.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            TravelToShoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(25.000, 60.000), new Pose(48.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(48.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                    .build();

            ArtifactSetup3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(42.000, 35.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                    .build();

            ArtifactPickup3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(42.000, 35.000), new Pose(25.000, 35.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            TravelToShoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(25.000, 35.000), new Pose(48.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            Shoot4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(48.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                    .build();

            Park = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(38.500, 33.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();
        }
    }

    private Follower follower;
    private Timer pathTimer, opmodeTimer;

    private Paths paths;

    private int pathState;

    private DcMotor intakeLeft;
    // private DcMotor intakeRight; // not used in this autonomous mode

    // How long (ms) to wait at the shooting position before moving on
    private static final double SHOOT_WAIT_MS = 2000;

    // Reduced drive speed used during ArtifactPickup paths so the intake
    // has time to sweep the balls in as the robot slides left into them.
    // 1.0 = full speed, 0.35 = ~35% — tune this as needed.
    private static final double INTAKE_SWEEP_SPEED = 0.35;
    private static final double FULL_SPEED = 1.0;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // === SHOOT 1 (preload) — robot at (48, 96) ===
                intakeLeft.setPower(0);
                // TODO: Add shooting mechanism code here (e.g. shooter.fire())
                if (pathTimer.getElapsedTimeSeconds() * 1000 > SHOOT_WAIT_MS) {
                    follower.followPath(paths.ArtifactSetup1, true);
                    setPathState(1);
                }
                break;
            case 1:
                // ArtifactSetup — approaching row 1; turn on intake motors
                intakeLeft.setPower(1);
                // intakeRight.setPower(1); // not used in this autonomous mode
                if (!follower.isBusy()) {
                    // Slow down so the intake can sweep the balls in as we slide left
                    follower.setMaxPower(INTAKE_SWEEP_SPEED);
                    follower.followPath(paths.ArtifactPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                // ArtifactPickup — slowly sweeping balls into intake while sliding left
                intakeLeft.setPower(1);
                // intakeRight.setPower(1); // not used in this autonomous mode
                if (!follower.isBusy()) {
                    // Restore full speed for the return trip
                    follower.setMaxPower(FULL_SPEED);
                    follower.followPath(paths.TravelToShoot1, true);
                    setPathState(3);
                }
                break;
            case 3:
                // Travel back to shoot position
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                // === SHOOT 2 — robot at (48, 96) ===
                intakeLeft.setPower(0);
                // TODO: Add shooting mechanism code here (e.g. shooter.fire())
                if (pathTimer.getElapsedTimeSeconds() * 1000 > SHOOT_WAIT_MS) {
                    follower.followPath(paths.ArtifactSetup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                // ArtifactSetup — approaching row 2; turn on intake motors
                intakeLeft.setPower(1);
                // intakeRight.setPower(1); // not used in this autonomous mode
                if (!follower.isBusy()) {
                    // Slow down so the intake can sweep the balls in as we slide left
                    follower.setMaxPower(INTAKE_SWEEP_SPEED);
                    follower.followPath(paths.ArtifactPickup2, true);
                    setPathState(6);
                }
                break;
            case 6:
                // ArtifactPickup — slowly sweeping balls into intake while sliding left
                intakeLeft.setPower(1);
                // intakeRight.setPower(1); // not used in this autonomous mode
                if (!follower.isBusy()) {
                    // Restore full speed for the return trip
                    follower.setMaxPower(FULL_SPEED);
                    follower.followPath(paths.TravelToShoot2, true);
                    setPathState(7);
                }
                break;
            case 7:
                // Travel back to shoot position
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                // === SHOOT 3 — robot at (48, 96) ===
                intakeLeft.setPower(0);
                // TODO: Add shooting mechanism code here (e.g. shooter.fire())
                if (pathTimer.getElapsedTimeSeconds() * 1000 > SHOOT_WAIT_MS) {
                    follower.followPath(paths.ArtifactSetup3, true);
                    setPathState(9);
                }
                break;
            case 9:
                // ArtifactSetup — approaching row 3; turn on intake motors
                intakeLeft.setPower(1);
                // intakeRight.setPower(1); // not used in this autonomous mode
                if (!follower.isBusy()) {
                    // Slow down so the intake can sweep the balls in as we slide left
                    follower.setMaxPower(INTAKE_SWEEP_SPEED);
                    follower.followPath(paths.ArtifactPickup3, true);
                    setPathState(10);
                }
                break;
            case 10:
                // ArtifactPickup — slowly sweeping balls into intake while sliding left
                intakeLeft.setPower(1);
                // intakeRight.setPower(1); // not used in this autonomous mode
                if (!follower.isBusy()) {
                    // Restore full speed for the return trip
                    follower.setMaxPower(FULL_SPEED);
                    follower.followPath(paths.TravelToShoot3, true);
                    setPathState(11);
                }
                break;
            case 11:
                // Travel back to shoot position
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
                // === SHOOT 4 (final shot before park) — robot at (48, 96) ===
                intakeLeft.setPower(0);
                // TODO: Add shooting mechanism code here (e.g. shooter.fire())
                if (pathTimer.getElapsedTimeSeconds() * 1000 > SHOOT_WAIT_MS) {
                    follower.followPath(paths.Park, true);
                    setPathState(13);
                }
                break;
            case 13:
                // Driving to park
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                // Parked — done
                break;
        }
    }

    public void setPathState(int newPathState) {
        pathState = newPathState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(48.000, 96.000, Math.toRadians(135)));

        intakeLeft = hardwareMap.get(DcMotor.class, "intake_left");
        // intakeRight = hardwareMap.get(DcMotor.class, "intake_right"); // not used in this autonomous mode

        paths = new Paths(follower);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void loop() {
        follower.update();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}