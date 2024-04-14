package AlgoRun;

public class algoParam {
    public static double capacity;  //vehicle capacity
    public static double timelimit;  // hours of service limit
    public static double timehorizon;  // hours of service limit
    public static double FTL;  // TL distance break
    public static double RPM; // TL unit rate per mile
    public static double LUT; // base loading/unloading service time

    public static double MAXLT; // maximum time limit for labeling procedure
    public static int MAXL; // maximum negative labels for labeling procedure
    public static double MAXR; // maximum total negative labels for CG
    public static double MAXLAB; // maximum total labels for CG

    public static String Exper;
    public static String dataPath;      // 实验数据路径
    public static String RoutesFile;    // 路网数据文件名
    public static String ProvinceFile;  //省份读取文件

    public static String ResultPath;     // 文件输出路径
    public static String InstanceResultPath;   // 类型实例对应的输出路径
    public static String solPath;        // 每个实例对应的输出路径
    public static String csvPath;        // 记录所有实例结果的CSV文件输出路径
}

