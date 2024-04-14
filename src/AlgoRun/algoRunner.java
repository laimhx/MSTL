package AlgoRun;

import Common.*;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;

import static AlgoRun.algoParam.MAXLAB;


public class algoRunner {
    public void run( ) throws ParseException {
        algoParam.capacity = 33; //vehicle capacity
        algoParam.timehorizon = 48;
        algoParam.FTL = 300; //TL distance break
        algoParam.RPM = 50;  //TL unit rate per mile
        algoParam.LUT = 15;  //base loading/unloading service time: minutes

        algoParam.MAXLT = 60.0; //maximum time limit for labeling procedure: second
        algoParam.MAXL = 200; //maximum negative labels for labeling procedure
        algoParam.MAXR = 1e6; //maximum total negative labels for CG
        algoParam.MAXLAB = 1e8; // maximum total labels for CG

        algoParam.Exper = "SensJS"; //tasks: smallJS, medJS, JS, SensJS
        algoParam.ProvinceFile = "./datM/Centers.csv";
        algoParam.RoutesFile = "./datM/Routes.csv";

        switch (algoParam.Exper) {
            case "smallJS" -> {
                algoParam.dataPath = "./datM/smallJS";
                algoParam.timelimit = 12; //HOS
            }

            case "smallZJ" -> {
                algoParam.dataPath = "./datM/smallZJ";
                algoParam.timelimit = 16; //HOS
            }

            case "medJS" -> {
                algoParam.dataPath = "./datM/medJS";
                algoParam.timelimit = 12; //HOS
            }

            case "medZJ" -> {
                algoParam.dataPath = "./datM/medZJ";
                algoParam.timelimit = 16; //HOS
            }

            case "JS" -> {
                algoParam.dataPath = "./datM/JS";
                algoParam.timelimit = 12; //HOS
            }

            case "ZJ" -> {
                algoParam.dataPath = "./datM/ZJ";
                algoParam.timelimit = 16; //HOS
            }

            case "SensJS"-> {
                algoParam.dataPath = "./datM/SensJS";
                algoParam.timelimit = 12; //HOS
            }

            case "SensZJ"-> {
                algoParam.dataPath = "./datM/SensZJ";
                algoParam.timelimit = 16; //HOS
            }
        }

        algoParam.ResultPath = "./result";
        algoParam.InstanceResultPath = algoParam.ResultPath  + "/" + algoParam.dataPath.split("/")[2];
        algoParam.solPath = algoParam.InstanceResultPath + "/sol";
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd");
        algoParam.csvPath = algoParam.ResultPath + "/" + algoParam.Exper + "_" + sdf.format(new Date()) + ".csv";

        makeResultFolders(); // 创建文件目录
        writeCSV(makeCSVTitle(),true); // 创建所有实例记录CSV文件

        //start experiment runs
        System.out.println("********* Get started with instance set: " + algoParam.Exper + " *********");
        Instance[] Insts = readInsts();
        //tasks: ExperJS, ExperZJ, SensJS, SensZJ
        if (algoParam.Exper.equals("SensJS") || algoParam.Exper.equals("SensZJ")) {
            runAlgoSense(Insts);
        } else {
            runAlgo(Insts);
        }
    }


    void runAlgo(Instance[] instances) throws ParseException {
        for (Instance inst : instances) {
            long startTime = System.currentTimeMillis();
            inst.initial_data();

            algo test = new algo(inst);
            test.run();
            writeResult(inst);

            //record running time
            long endTime = System.currentTimeMillis();
            double tt = (endTime-startTime)/1000.0/60.0;
            System.out.printf("********** Test %s with orders %d is completed with time %.2f min **********%n",
                    inst.inst_name, inst.NUM, tt);
            System.out.println("-----------------------------------------------------------");
        }
    }


    void runAlgoSense(Instance[] instances) throws ParseException {
        ArrayList<Integer> MaxOrder = new ArrayList<>(Arrays.asList(2, 4, 6));
        ArrayList<Double> Radius = new ArrayList<>(Arrays.asList(40.0, 80.0, 120.0));
        ArrayList<Double> Stops = new ArrayList<>(Arrays.asList(400.0, 800.0, 1200.0));
        ArrayList<Double> Detours = new ArrayList<>(Arrays.asList(5.0, 10.0, 15.0));

        long startTime, endTime, t0, t1;
        double tt;

        //factor = bundle, radius, stop, detour
        t0 = System.currentTimeMillis();

        for (Instance inst : instances) {
            startTime = System.currentTimeMillis();
            inst.initial_data();

            //base case
            System.out.println("*********** Start test on the base case ***********");
            inst.LIFO = false;
            inst.maxOrders = MaxOrder.get(1);
            inst.radius = Radius.get(0);
            inst.stop = Stops.get(0);
            inst.detour = Detours.get(0);
            algo test0 = new algo(inst);
            test0.run();
            writeResult(inst);

            //LIFO
            System.out.printf("*********** Start test on factor LIFO = %b *********** %n",
                    inst.LIFO);
            inst.LIFO = true;
            inst.maxOrders = MaxOrder.get(1);
            inst.radius = Radius.get(0);
            inst.stop = Stops.get(0);
            inst.detour = Detours.get(0);
            algo test1 = new algo(inst);
            test1.run();
            writeResult(inst);

            //bundle
            System.out.printf("*********** Start test on factor bundle size B = %d *********** %n",
                    MaxOrder.get(1));
            inst.LIFO = false;
            inst.maxOrders = MaxOrder.get(0);
            inst.radius = Radius.get(0);
            inst.stop = Stops.get(0);
            inst.detour = Detours.get(0);
            algo test2a = new algo(inst);
            test2a.run();
            writeResult(inst);

            System.out.printf("*********** Start test on factor bundle size B = %d *********** %n",
                    MaxOrder.get(2));
            inst.LIFO = false;
            inst.maxOrders = MaxOrder.get(2);
            inst.radius = Radius.get(0);
            inst.stop = Stops.get(0);
            inst.detour = Detours.get(0);
            algo test2b = new algo(inst);
            test2b.run();
            writeResult(inst);

            //radius
            System.out.printf("*********** Start test on factor radius E = %.2f *********** %n",
                    Radius.get(1));
            inst.LIFO = false;
            inst.maxOrders = MaxOrder.get(1);
            inst.radius = Radius.get(1);
            inst.stop = Stops.get(0);
            inst.detour = Detours.get(0);
            algo test3a = new algo(inst);
            test3a.run();
            writeResult(inst);

            System.out.printf("*********** Start test on factor radius E = %.2f *********** %n",
                    Radius.get(2));
            inst.LIFO = false;
            inst.maxOrders = MaxOrder.get(1);
            inst.radius = Radius.get(2);
            inst.stop = Stops.get(0);
            inst.detour = Detours.get(0);
            algo test3b = new algo(inst);
            test3b.run();
            writeResult(inst);

            //stop
            System.out.printf("*********** Start test on factor stop-off f = %.2f *********** %n",
                    Stops.get(1));
            inst.LIFO = false;
            inst.maxOrders = MaxOrder.get(1);
            inst.radius = Radius.get(0);
            inst.stop = Stops.get(1);
            inst.detour = Detours.get(0);
            algo test4a = new algo(inst);
            test4a.run();
            writeResult(inst);

            System.out.printf("*********** Start test on factor stop-off f = %.2f *********** %n",
                    Stops.get(2));
            inst.LIFO = false;
            inst.maxOrders = MaxOrder.get(1);
            inst.radius = Radius.get(0);
            inst.stop = Stops.get(2);
            inst.detour = Detours.get(0);
            algo test4b = new algo(inst);
            test4b.run();
            writeResult(inst);

            //detour
            System.out.printf("*********** Start test on factor detour h = %.2f *********** %n",
                    Detours.get(1));
            inst.LIFO = false;
            inst.maxOrders = MaxOrder.get(1);
            inst.radius = Radius.get(0);
            inst.stop = Stops.get(0);
            inst.detour = Detours.get(1);
            algo test5a = new algo(inst);
            test5a.run();
            writeResult(inst);

            System.out.printf("*********** Start test on factor detour h = %.2f *********** %n",
                    Detours.get(2));
            inst.LIFO = false;
            inst.maxOrders = MaxOrder.get(1);
            inst.radius = Radius.get(0);
            inst.stop = Stops.get(0);
            inst.detour = Detours.get(2);
            algo test5b = new algo(inst);
            test5b.run();
            writeResult(inst);

            //record running time
            endTime = System.currentTimeMillis();
            tt = (endTime - startTime)/1000.0/60.0;
            System.out.printf("********** Test %s with orders %d is completed with time %.2f min **********%n",
                    inst.inst_name, inst.NUM, tt);
            System.out.println("-----------------------------------------------------------");
        }

        t1 = System.currentTimeMillis();
        tt = (t1 - t0)/1000.0/3600.0;
        System.out.printf("********** Sensitivity tests completed with time %.2f hrs **********%n", tt);
    }


    // 文件夹下所有实例读取
    Instance[] readInsts(){
        File dir = new File(algoParam.dataPath);
        File[] files = dir.listFiles();
        assert files != null;

        Instance[] instances = new Instance[files.length];
        for(int i = 0; i < files.length; i++){
            instances[i] = new Instance(files[i].getName());
        }

        return instances;
    }


    public static void makeResultFolders() {
        File resultFolder = new File(algoParam.ResultPath);
        if (!resultFolder.exists() || !resultFolder.isDirectory())
            resultFolder.mkdirs();

        File algoFolder = new File(algoParam.InstanceResultPath);
        if (!algoFolder.exists() || !algoFolder.isDirectory())
            algoFolder.mkdir();

        File solFolder = new File(algoParam.solPath);
        if (solFolder.exists() || !solFolder.isDirectory())
            solFolder.mkdir();
    }


    String makeCSVTitle() {
        String str;
        str = "inst_name, netCase, NUM, numV, numE, " +
                "LIFO, maxOrders, radius, stop, detour, " +
                "OriginCost, numVar, numConstr, " +
                "LTL_exact, MSTL_exact_2, MSTL_exact_3, MSTL_exact_4, MSTL_exact_5M, " +
                "feasb_exact, COST_exact, TIME_exact, gap_exact, " +
                "EnumNum, Two_route, Three_route, Four_route, Five_route, " +
                "LTL_enum, MSTL_enum_2, MSTL_enum_3, MSTL_enum_4, MSTL_enum_5M, " +
                "COST_enum, ITER_enum, TIME_enum, " +
                "OPTNum, LTL_Opt, MSTL_Opt_2, MSTL_Opt_3, MSTL_Opt_4, MSTL_Opt_5M, " +
                "COST_Opt, TIME_Opt, Swap_Opt, ITER_Opt, " +
                "RCGNum, RCG_2, RCG_3, RCG_4, RCG_5M, " +
                "LTL_label, MSTL_label_2, MSTL_label_3, MSTL_label_4, MSTL_label_5M, " +
                "COST_label, ITER_label, TIME_label, impr_label, gap_label, " +
                "gap_LP_label, TIME_LP_label, TIME_SP_label, TIME_IP_label, CG_label";
        return str;
    }


    public static void writeCSV(String text, boolean title) {
        File csvFile = new File(algoParam.csvPath);
        try {
            if (!csvFile.exists()) {
                csvFile.createNewFile();
                write(csvFile, text);
            } else {
                if (!title) append(csvFile, text);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    public static void write(File file, String text) {
        try {
            FileWriter fw = new FileWriter(file);
            fw.write(text + "\r\n");
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    public static void append(File file, String text) {
        try {
            FileWriter fw = new FileWriter(file, true);
            fw.write(text + "\r\n");
            fw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    public static void writeResult(Instance instance){
        //Output the summary result csv file
        writeCSV(instance.makeCsvItem(), false);

        //Output the route solution in json file
        /*String solPath = algoParam.solPath + "/" + instance.inst_name + ".json";
        File solFile = new File(solPath);
        write(solFile, instance.resultToString());*/
    }

}
