package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BlueAutonomous", group = "Autonomous")
public class BlueAutonomous extends OpMode {

    // Step callback interface

    /**
     * A single autonomous step: run onEnter() once, then tick isDone() each loop.
     */
    private interface Step {
        /**
         * Called exactly once when this step becomes active.
         */
        void onEnter();

        /**
         * Called every loop while this step is active. Return true to advance.
         */
        boolean isDone();

        /**
         * Human-readable label shown in telemetry.
         */
        String label();
    }

    // Paths (export from visualizer)
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
                    .addPath(new BezierLine(new Pose(42.000, 85.000), new Pose(18.000, 85.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            TravelToShoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(18.000, 85.000), new Pose(48.000, 96.000)))
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
                    .addPath(new BezierLine(new Pose(42.000, 60.000), new Pose(18.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            TravelToShoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(18.000, 60.000), new Pose(48.000, 96.000)))
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
                    .addPath(new BezierLine(new Pose(42.000, 35.000), new Pose(18.000, 35.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            TravelToShoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(18.000, 35.000), new Pose(48.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            Shoot4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(48.000, 96.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                    .build();

            Park = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(48.000, 96.000), new Pose(105.500, 33.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .build();
        }
    }

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private Paths paths;

    private DcMotor intakeLeft;
    // private DcMotor intakeRight; // not used in this autonomous mode

    private DcMotorEx flywheelLeft;
    private DcMotorEx flywheelRight;

    // How long to wait at the shooting position before moving on
    private static final double SHOOT_WAIT_MS = 2000;

    // Reduced drive speed used during ArtifactPickup paths so the intake sweeps balls correctly
    private static final double INTAKE_SWEEP_SPEED = 0.45;
    private static final double FULL_SPEED = 1.0;

    // NOTE: A is used when there are 3 balls / first shot; B for 2 or fewer balls
    // TODO: Switch this to velocity
    private static final double FLYWHEEL_TARGET_POWER_A = 0.8;
    private static final double FLYWHEEL_TARGET_VELOCITY_B = 1200.0;

    private int ballCount = 3;

    // Step-sequencer state
    private List<Step> sequence;
    private int currentStep;
    private boolean stepEntered;

    /**
     * Advance to the next step (or stay on the last one when done).
     */
    private void nextStep() {
        currentStep = Math.min(currentStep + 1, sequence.size() - 1);
        stepEntered = false;
        pathTimer.resetTimer();
    }

    /**
     * Build a step that follows a path at the given max-power, with optional side-effects on enter.
     */
    private Step followStep(String name, PathChain path, double maxPower, Runnable onEnterAction) {
        return new Step() {
            @Override
            public void onEnter() {
                follower.setMaxPower(maxPower);
                follower.followPath(path, true);
                if (onEnterAction != null) onEnterAction.run();
            }

            @Override
            public boolean isDone() {
                return !follower.isBusy();
            }

            @Override
            public String label() {
                return name;
            }
        };
    }

    /**
     * Build a timed wait step (runs onEnterAction once, then waits SHOOT_WAIT_MS).
     */
    private Step waitStep(String name, Runnable onEnterAction) {
        return new Step() {
            @Override
            public void onEnter() {
                if (onEnterAction != null) onEnterAction.run();
            }

            @Override
            public boolean isDone() {
                return pathTimer.getElapsedTimeSeconds() * 1000 > SHOOT_WAIT_MS;
            }

            @Override
            public String label() {
                return name;
            }
        };
    }

    // Sequence definition
    private void buildSequence() {
        sequence = new ArrayList<>();

        // Initial shot (balls already loaded)
        sequence.add(waitStep("Shoot 1", () -> {
            setLeftIntakeState(false);
            shootAll();
        }));

        // Row 1
        sequence.add(followStep("Setup -> Row 1", paths.ArtifactSetup1, FULL_SPEED, () -> {
            shutdownFlywheel();
            setLeftIntakeState(true);
        }));
        sequence.add(followStep("Pickup Row 1", paths.ArtifactPickup1, INTAKE_SWEEP_SPEED, null));
        sequence.add(followStep("Return -> Shoot 2", paths.TravelToShoot1, FULL_SPEED, () -> {
            ballCount = 3;
            spinupFlywheel();
        }));
        sequence.add(waitStep("Shoot 2", () -> {
            setLeftIntakeState(false);
            shootAll();
        }));

        // Row 2
        sequence.add(followStep("Setup -> Row 2", paths.ArtifactSetup2, FULL_SPEED, () -> {
            shutdownFlywheel();
            setLeftIntakeState(true);
        }));
        sequence.add(followStep("Pickup Row 2", paths.ArtifactPickup2, INTAKE_SWEEP_SPEED, null));
        sequence.add(followStep("Return -> Shoot 3", paths.TravelToShoot2, FULL_SPEED, () -> {
            ballCount = 3;
            spinupFlywheel();
        }));
        sequence.add(waitStep("Shoot 3", () -> {
            setLeftIntakeState(false);
            shootAll();
        }));

        // Row 3
        sequence.add(followStep("Setup -> Row 3", paths.ArtifactSetup3, FULL_SPEED, () -> {
            shutdownFlywheel();
            setLeftIntakeState(true);
        }));
        sequence.add(followStep("Pickup Row 3", paths.ArtifactPickup3, INTAKE_SWEEP_SPEED, null));
        sequence.add(followStep("Return -> Shoot 4", paths.TravelToShoot3, FULL_SPEED, () -> {
            ballCount = 3;
            spinupFlywheel();
        }));
        sequence.add(waitStep("Shoot 4", () -> {
            setLeftIntakeState(false);
            shootAll();
        }));

        // Park
        sequence.add(followStep("Park", paths.Park, FULL_SPEED, null));
        sequence.add(new Step() {
            @Override
            public void onEnter() { /* done, nothing to do */ }

            @Override
            public boolean isDone() {
                return false;
            } // stay here forever

            @Override
            public String label() {
                return "Parked";
            }
        });
    }

    public void autonomousPathUpdate() {
        if (sequence == null || currentStep >= sequence.size()) return;

        Step step = sequence.get(currentStep);

        if (!stepEntered) {
            step.onEnter();
            stepEntered = true;
        }

        if (step.isDone()) {
            nextStep();
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(48.000, 96.000, Math.toRadians(135)));

        intakeLeft = hardwareMap.get(DcMotor.class, "intake_left");
        // intakeRight = hardwareMap.get(DcMotor.class, "intake_right"); // not used

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "right_flywheel");

        paths = new Paths(follower);
        buildSequence();
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        currentStep = 0;
        stepEntered = false;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Telemetry
        Step active = (sequence != null && currentStep < sequence.size())
                ? sequence.get(currentStep) : null;
        telemetry.addData("step", "%d / %d  -  %s",
                currentStep + 1, sequence != null ? sequence.size() : 0,
                active != null ? active.label() : "-");
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("balls", ballCount);
        telemetry.update();
    }

    private void setLeftIntakeState(boolean on) {
        intakeLeft.setPower(on ? 1 : 0);
    }

    private void setFlywheelVelocity(double targetVelocity) {
        flywheelLeft.setVelocity(targetVelocity);
        flywheelRight.setVelocity(targetVelocity);
    }

    private void setFlywheelPower(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }

    private void shutdownFlywheel() {
        flywheelLeft.setPower(0);
        flywheelRight.setPower(0);
    }

    private void spinupFlywheel() {
        if (ballCount > 2) {
            setFlywheelPower(FLYWHEEL_TARGET_POWER_A);
        } else {
            setFlywheelVelocity(FLYWHEEL_TARGET_VELOCITY_B);
        }
    }

    private void shoot() {
        spinupFlywheel();
        // TODO: Wait for spin up and activate servo to push ball into flywheel
        ballCount -= 1;
    }

    // TODO: Delay needs to be re-reviewed
    private void shootAll() {
        while (ballCount > 0) {
            shoot();

            // Sleep for 2s

        }
    }
}

