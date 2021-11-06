public  enum ArmShoulderPositions {

    INTAKE(50,0.3,0.23,0.77),
    LEVEL1(1100,0.4,0.2,0.8),
    LEVEL2(860,0.4,0.4,0.6),
    LEVEL3(600,0.4,0.75,0.25),
    TSE_DROP(600,0.4,0.54,0.46),
    GROUND_PICKUP(1000,0.4,0.5,0.5);


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