package frc.lib.math;

public class ArmInverseKinematics {
    private static ArmLocation location;
    private static ArmAngles angles;

    public static final double ARM_SECTION_LENGTH = 1;
    public static void main(String args[]){
        double[] to = {1.5,0,0};
        location = new ArmLocation(to);
        double[] spher = location.toSpherical();
        System.out.println(spher[0] + " " + spher[1] + " " + spher[2]);
        calc();
    }

    /**
     * sets location of arm in robot-relative coordinates
     * @param newLocation an ArmLocation containing the desired coordinates of the end affector
     */
    public static void setLocation(ArmLocation newLocation){
        location = newLocation;
        calc();
    }

    /**
     * get calculated result
     * @return ArmAngles object containing 
     */
    public static ArmAngles getAngles(){
        return angles;
    }
   
    /**
     * runs all calculations; takes whatever is in ArmLocation location, and runs calculations on it, and puts the result in ArmAngles angles
     */
    private static void calc() {
        double[] point = location.toSpherical(); //get spherical coords as [dist, ground angle, height angle] or [p theta t] darn symbols

        double posTranslationJointTotalDist = point[0];
        double posAngleJoint = point[1];
        double posTranslationJointHeightMod = point[2];

        //create new armangles to start adding to...
        ArmAngles newAngles = new ArmAngles();
        
        newAngles.posRotationJoint = posAngleJoint;

        //get angles for the joints responsible for arm translation - these would be the second and third joints on most arms. 
        double[] translBaseA = findTranslationBaseAngles(posTranslationJointTotalDist);
        newAngles.posTranslationJoint1 = translBaseA[0] + posTranslationJointHeightMod;
        newAngles.posTranslationJoint2 = translBaseA[1];
        angles = newAngles;
        System.out.println(translBaseA[0] + posTranslationJointHeightMod + " " + translBaseA[1]);
    }

    /**
     * gets the angles for the joints responsible for the arm extension distance, usually the second/third joint.
     * @param dist distance between arm base and end affector
     * @return array with [first joint angle, second joint angle]
     */
    private static double[] findTranslationBaseAngles(double dist){
        var d = dist/2; //dist to first joint
        //first section of arm:
        var y_coord = Math.sqrt(ARM_SECTION_LENGTH - (d*d));
        var slope = y_coord / d;
        var angle = Math.atan(slope);

        //second section of arm:
        var secondAngle = Math.toRadians(180 - (Math.toDegrees(angle) * 2)); // iscoscilese triangle
        return new double[]{angle, secondAngle};
    }

    public static class ArmLocation {
        public double[] endAffectorLocation = new double[3];
        public double[] toSpherical(){
            double x = -endAffectorLocation[0],
            y = -endAffectorLocation[1],
            z = endAffectorLocation[2];
            double p = Math.sqrt((x*x) + (y*y) + (z*z));
            double theta = Math.PI + Math.atan2(y,x);
            double t = Math.acos(z/Math.sqrt((x*x) + (y*y) + (z*z)));
            return new double[]{p,theta,t};
        }
        public ArmLocation(double[] location) {endAffectorLocation = location;}
    }
    public static class ArmAngles{
        public double posRotationJoint,
                  posTranslationJoint1,
                  posTranslationJoint2;
        public double endRotationJoint1,
                  endRotationJoint2,
                  endRotationJoint3;
    }
}
