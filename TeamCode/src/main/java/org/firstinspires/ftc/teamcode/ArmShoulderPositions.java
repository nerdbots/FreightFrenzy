package org.firstinspires.ftc.teamcode;

public  enum ArmShoulderPositions {

    INTAKE(0,0.2,0.28,0.72),
    LEVEL1(1100,0.4,0.28,0.72),
    LEVEL2(830,0.3,0.53,0.47),
    LEVEL3(600,0.3,0.7,0.3),
    TSE_DROP(600,0.3,0.62,0.38),
    GROUND_PICKUP(1100,0.3,0.43,0.57),
    HOME(100,0.3,0.15,0.85),
    SHARED_HUB(1000, 0.4, 0.38,0.62);


    private  final int armTarget;
    private final double maxPower;
    private final double leftWristServoPosition;
    private final double rightWristServoPosition;


    private ArmShoulderPositions(int armTarget, double maxPower, double leftWristServoPosition, double rightWristServoPosition) {
        this.armTarget = armTarget;
        this.maxPower = maxPower;
        this.leftWristServoPosition = leftWristServoPosition;
        this.rightWristServoPosition = rightWristServoPosition;

    }

    public int getArmTarget() {
        return this.armTarget;
    }

    public double getMaxPower() {
        return this.maxPower;
    }

    public double getLeftWristServoPosition() {
        return this.leftWristServoPosition;
    }
    public double getRightWristServoPosition() {
        return this.rightWristServoPosition;
    }

}