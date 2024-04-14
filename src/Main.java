import AlgoRun.algoRunner;
import java.text.ParseException;

public class Main{
    public static void main(String[] args) throws ParseException {
        long startTime = System.currentTimeMillis();

        //Computational task: ExperJS, ExperZJ, Sensitivity
        new algoRunner().run();

        long endTime = System.currentTimeMillis();
        double time = (endTime-startTime)/1000.0/3600.0;
        System.out.printf("All instances are completed with time: %.2f hours", time);
    }
}
