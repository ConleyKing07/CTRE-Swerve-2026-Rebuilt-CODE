package frc.robot.vision;

public class ShooterMath {

    // distance meters , rpm
    private static final double[][] table = {
        {1.75, 2100},
        {2.0, 2200},
        {2.5, 2400},
        {3.0, 2600},
        {3.5, 2800},
        {4.0, 3000},
        {4.5, 3200}
    };

    public static double getRPM(double distance){

        if(distance <= table[0][0])
            return table[0][1];

        for(int i=0;i<table.length-1;i++){

            double d1=table[i][0];
            double d2=table[i+1][0];

            if(distance>=d1 && distance<=d2){

                double r1=table[i][1];
                double r2=table[i+1][1];

                double t=(distance-d1)/(d2-d1);

                return r1+(r2-r1)*t;
            }
        }

        return table[table.length-1][1];
    }
}
