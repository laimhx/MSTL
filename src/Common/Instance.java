package Common;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;

import static AlgoRun.algoParam.*;

public class Instance {
    public String orderfile;
    public String StartDepot;
    public String returnDepot;

    //bundling parameters
    public boolean LIFO;
    public int maxOrders;
    public double radius;
    public double stop;
    public double detour;
    public boolean large;

    public HashMap<Integer, Customer> AllCustomers;
    public Distance[][] Graph;
    public double[][] Arc;
    public double[][] Rate;
    public double[] SignleCost;

    //Output data
    public String inst_name;
    public String netCase;
    public int NUM;
    public int numV;
    public int numE;
    public double OriginCost;

    //Exact algorithm
    public int numVar; //unit: 1
    public int numConstr; //unit: 1
    public int LTL_exact;
    public int MSTL_exact_2;
    public int MSTL_exact_3;
    public int MSTL_exact_4;
    public int MSTL_exact_5M;
    public int feasb_exact;
    public double COST_exact;
    public double TIME_exact; //unit: minutes
    public double gap_exact;

    //Enumeration algorithm
    public int EnumNum;
    public int Two_route;
    public int Three_route;
    public int Four_route;
    public int Five_route;
    public int LTL_enum;
    public int MSTL_enum_2;
    public int MSTL_enum_3;
    public int MSTL_enum_4;
    public int MSTL_enum_5M;
    public double COST_enum;
    public double ITER_enum; //unit: 10^8, total number of iterations
    public double TIME_enum; //unit: minutes

    //2-Opt heuristic algorithm
    public int OPTNum; //number of initialized routes
    public int LTL_Opt;
    public int MSTL_Opt_2;
    public int MSTL_Opt_3;
    public int MSTL_Opt_4;
    public int MSTL_Opt_5M;
    public double COST_Opt;
    public double TIME_Opt; //unit: minutes
    public int Swap_Opt; //total number of successful swap operations
    public int ITER_Opt; //total number of search iterations

    // Column generation algorithm
    public int RCGNum; //number of generated routes
    public int RCG_2;
    public int RCG_3;
    public int RCG_4;
    public int RCG_5M;
    public int LTL_label;
    public int MSTL_label_2;
    public int MSTL_label_3;
    public int MSTL_label_4;
    public int MSTL_label_5M;
    public double COST_label;
    public int ITER_label;
    public double TIME_label; //unit: minutes
    public double impr_label;
    public double gap_label;
    public double gap_LP_label; //the final LP relaxation gap
    public double TIME_LP_label; //unit: minutes, the total time for solving LP relaxation
    public double TIME_SP_label; //unit: minutes, the total time for labelling procedure
    public double TIME_IP_label; //unit: minutes, the time for solving final IP
    public double CG_label; //unit: 10^6, number of all generated labels

    //The solution routes
    public HashMap<Integer,Route> routesExact;
    public HashMap<Integer,Route> routesEnmu;
    public HashMap<Integer,Route> routesOpt;
    public HashMap<Integer,Route> routesLabel;

    //MSTL routes with negative reduced cost
    public ArrayList<Route> CGRoutes;


    public Instance(String orderfile) {
        this.orderfile = orderfile;
    }


    public void initial_data() throws ParseException {
        long t0 = System.currentTimeMillis();

        //orderfile: JS_400_1.csv; ZJ_400_1.csv; JS_1.csv; ZJ_1.csv
        String[] str = orderfile.split("\\.");
        inst_name = str[0];
        String[] st = inst_name.split("_");
        netCase = st[0]; //JS, ZJ

        //Read order data
        load_orders(orderfile);

        //Read road network data
        load_distance(RoutesFile);

        //Build shipping network
        buildNetwork();

        //Check scale of the instance: 500
        large = (NUM >= 1000);
        long t1 = System.currentTimeMillis();
        double tt = (t1-t0)/1000.0/60.0;
        System.out.printf("-------- Data of %s in %s case is loaded with time %.2f min--------%n",
                inst_name, netCase, tt);

        //LTL costs
        SignleCost = new double[NUM + 1];
        OriginCost = 0;
        for (Customer cc : AllCustomers.values()) {
            if (cc.id >= 1 && cc.id <= NUM) {
                SignleCost[cc.id] = cc.singleCost;
                OriginCost += cc.singleCost;
            }
        }

        //Double-check LTL cost no larger than TL rates alone!
        for (int i = 1; i <= NUM; i++) {
            double tl = Rate[i][NUM+i]*Graph[i][NUM+i].distance;
            if (SignleCost[i] - tl > -1e-5) {
                SignleCost[i] = 0.8 * tl;
            }
        }

        ParamsInitial();
        if (!check_data()) {
            System.out.println("########### Error in reading data! ###########");
            System.exit(0);
        }
    }


    //Read order data: node set customers include both picks and delis
    public void load_orders(String filename) throws ParseException {
        AllCustomers = new HashMap<>();
        String origin, destination;
        int origin_id, destination_id;
        double weight, serve;
        double early, last, singleCost;

        try{
            File file = new File(dataPath + "/" + filename);
            FileInputStream in = new FileInputStream(file);
            InputStreamReader fr = new InputStreamReader(in, "gbk");
            BufferedReader br = new BufferedReader(fr);
            String line;
            br.readLine();
            line = br.readLine();

            String[] params = line.split(",");
            StartDepot = params[0].replace("\"", "");
            returnDepot =params[1].replace("\"", "");
            NUM = Integer.parseInt(params[2]);
            LIFO = (Integer.parseInt(params[3]) > 0);
            maxOrders = Integer.parseInt(params[4]);
            radius = Double.parseDouble(params[5]);
            stop = Double.parseDouble(params[6]);
            detour = Double.parseDouble(params[7]);

            //Add depot node 0
            AllCustomers.put(0,new Customer(0,StartDepot,0,0.0,0.0,0,0,0,0));
            int node_id = 1;

            br.readLine();
            br.readLine();
            while ((line = br.readLine()) != null) {
                String[] items = line.split(",");
                origin = items[1].replace("\"","");
                origin_id = node_id;

                destination = items[3].replace("\"","");
                destination_id = node_id + NUM;
                node_id ++;
                weight = Double.parseDouble(items[7]);

                //loading and unloading service time: LUT = 15 min
                if (weight <= 10) {
                    serve = LUT;
                } else if (weight <= 15) {
                    serve = 2*LUT;
                } else {
                    serve = 3*LUT;
                }

                //pickup earliest time: items[9]; latest delivery time: items[11]
                early = (double)(items[8].charAt(items[8].length()-1))-'0' + Double.parseDouble(items[9].split(":")[0]);
                last = (double)(items[10].charAt(items[10].length()-1))-'0' + Double.parseDouble(items[11].split(":")[0]);

                singleCost = Double.parseDouble(items[12]);
                Customer node_pick = new Customer(origin_id,origin,weight,early,timehorizon,serve,singleCost,0,0); // 严格时间窗口
                Customer node_deli = new Customer(destination_id,destination,-weight,0.0,last,serve,singleCost,0,0);
                AllCustomers.put(origin_id,node_pick);
                AllCustomers.put(destination_id,node_deli);
            }

            //Add depot node 2N+1
            AllCustomers.put(2*NUM+1, new Customer(2*NUM+1,returnDepot, 0, 0.0, 0.0,
                    0,0,0,0));

            //end reading
            br.close();
            fr.close();
            in.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

        System.out.printf("--------Complete reading order data of %s: %d orders--------%n",
                inst_name, NUM);
    }


    // 读取routes.csv文件,生成当前距离矩阵
    public void load_distance(String filename){
        HashMap<String[], Distance> distance_map = new HashMap<>();
        String origin, destination;
        double distance, duration;

        try{
            File file = new File(filename);
            FileInputStream in = new FileInputStream(file);
            InputStreamReader fr = new InputStreamReader(in, "utf-8");
            BufferedReader br = new BufferedReader(fr);
            String line;
            br.readLine();
            Graph = new Distance[2*NUM+2][2*NUM+2];
            Rate = new double[2*NUM+2][2*NUM+2];

            while ((line = br.readLine()) != null) {
                String[] items = line.split(",");
                origin = items[0].replace("\"","");
                destination = items[1].replace("\"","");
                distance = Double.parseDouble(items[2]);
                duration = Double.parseDouble(items[3]);
                String[] location = new String[]{origin,destination};
                Distance dis = new Distance(location,distance,duration);
                distance_map.put(location, dis);
            }

            for (Customer node1:AllCustomers.values()) {
                String location1 = node1.pick_or_deli; //physical location
                for (Customer node2:AllCustomers.values()) {
                    String location2 = node2.pick_or_deli; //physical location
                    if (location1.equals(location2)) {
                        Graph[node1.id][node2.id] = new Distance(new String[]{location1,location2},0,0);
                        Rate[node1.id][node2.id] = 0;
                    } else {
                        for (String[] key: distance_map.keySet()) {
                            if (key[0].equals(location1) && key[1].equals(location2)) {
                                Graph[node1.id][node2.id] = distance_map.get(key);

                                //Set the line-haul rate per km: FTL = 250
                                if (Graph[node1.id][node2.id].distance <= FTL) {
                                    Rate[node1.id][node2.id] = RPM;
                                } else {
                                    Rate[node1.id][node2.id] = 0.9*RPM;
                                }

                                break;
                            }
                        }
                    }
                }
            }
            //end
            br.close();
            fr.close();
            in.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }


    public void buildNetwork() {
        numV = 2*NUM+2;
        numE = 0;

        Arc = new double[2*NUM+2][2*NUM+2];
        int v;
        for (int i = 0; i <= 2*NUM+1; i++) {
            for (int j = 0; j <= 2*NUM+1; j++) {
                v = 0;

                boolean BE = (i > 0 & j > 0 & i <= NUM & j <= NUM) || (i > NUM & j > NUM & i < 2*NUM+1 & j < 2*NUM+1);
                if (i != j && BE && Graph[i][j].distance <= radius){ //arc (i,j) or (n+i, n+j)
                    v = 1;
                }

                if (i == 0 && j > 0 && j <= NUM) { //arc (0, j)
                    v = 1;
                }

                if (i > NUM && i < 2*NUM+1 && j == 2*NUM+1) { //arc (n+i, 2n+1)
                    v = 1;
                }

                if (i > 0 && i <= NUM && j > NUM && (j < 2*NUM+1)) { //arc (i,n+j)
                    v = 1;
                }

                if (i == 0 && j == 2*NUM+1) { //dummy arc (0, 2n+1)
                    v = 1;
                }

                Arc[i][j] = 1.0*v;
                numE += v;
            }

            if (Arc[i][i] > 0) {
                System.err.printf("####### Error in the arc incidence matrix for node %d ####### %n", i);
            }
        }

        //End
        System.out.printf("-------- Complete the shipping network data: %d nodes, %d arcs --------%n",
                numV, numE);
    }


    public boolean check_data( ){
        if ((NUM > 0) && (maxOrders > 0) && (radius > 0) && (stop > 0) && (detour > 0)) {
            return true;
        } else {
            System.out.printf("########### Error in reading data: %d orders, %b LIFO, %d limit, %.2f km, %.2f stop-off, %.2f detour ########### %n",
                    NUM, LIFO, maxOrders, radius, stop, detour);
            return false;
        }
    }


    //Check feasibility conditions of a route
    public boolean bundle_feasible(Route route){
        int n = route.R.size();

        //LTL route
        if (n <= 4) {
            return true;
        }

        //MSTL route: check depots
        if (route.R.get(0) !=0 || route.R.get(n-1) != 2*NUM + 1) {
            return false;
        }

        //MSTL route: bundling and time constraints
        if (n > 2*maxOrders + 2) { //bundling size limit
            return false;
        }

        for (int i = 1; i <= n/2 - 2; i++) { //bundling radius constraints for pickup
            if (Graph[route.R.get(i)][route.R.get(i + 1)].distance > radius) {
                return false;
            }
        }
        for (int i = n/2; i <= n - 3; i++) { //bundling radius constraints for delivery
            if (Graph[route.R.get(i)][route.R.get(i + 1)].distance > radius) {
                return false;
            }
        }

        //If all feasible
        return true;
    }


    //Use whenever no need to check bundling radius limit
    public boolean feasible(Route route,boolean check) {
        //Initial values
        double load = 0, dist = 0, time = 0, subt = 0, cost = 0, epsl = 1e-5;

        //Empty routes
        if(route.R.size() <= 2) {
            if (!check) {
                route.load = load;
                route.dist = dist;
                route.time = time;
                route.subt = subt;
                route.cost = cost;
            }
            return true;
        }

        // Set the load
        for(int i = 1; i < route.R.size()/2; i++){
            load = load + AllCustomers.get(route.R.get(i)).demand;
        }
        if (load > capacity) {
            System.err.printf("###### Error route in load capacity: %.2f, %.2f ###### %n", load, capacity);
            return false;
        }

        //Set the distance
        for (int i = 1; i < route.R.size() - 2; i++) {
            dist += Graph[route.R.get(i)][route.R.get(i + 1)].distance;
        }

        //Set the service times
        if (route.R.size() == 4) { //LTL route
            cost = SignleCost[route.R.get(1)];
            for(int i = 1; i < route.R.size(); i++){
                int before = route.R.get(i-1);
                int current = route.R.get(i);
                time += Graph[before][current].duration + AllCustomers.get(before).serve/60;
            }
        } else { //MSTL route
            cost = Graph[route.R.get(1)][route.R.get(route.R.size()-2)].distance *
                    Rate[route.R.get(1)][route.R.get(route.R.size() - 2)] + (route.R.size() - 4) * stop +
                    (dist - Graph[route.R.get(1)][route.R.get(route.R.size()-2)].distance) * detour;

            //Time conditions
            double st = AllCustomers.get(route.R.get(1)).early - Graph[0][AllCustomers.get(route.R.get(1)).id].duration;
            route.setStartTime(st); //路径的出发时间自动定位为第一个订单的出发时间
            double arrive_time = route.startTime;

            //pickup arrival time
            for (int i = 1; i < route.R.size() / 2; i++) {
                int current = route.R.get(i), before = route.R.get(i - 1);
                arrive_time = arrive_time + Graph[before][current].duration + AllCustomers.get(before).serve/60;
                Customer customer = AllCustomers.get(current);
                if (arrive_time <= customer.early) {
                    arrive_time = customer.early;
                }
            }

            //delivery arrival time
            for (int i = route.R.size() / 2; i < route.R.size() - 1; i++) {
                int current = route.R.get(i), before = route.R.get(i - 1);
                arrive_time = arrive_time + Graph[before][current].duration + AllCustomers.get(before).serve/60;
                Customer customer = AllCustomers.get(current);
                if (arrive_time > customer.last) {
                    subt = Double.MAX_VALUE; //no allowing delay
                    return false;
                }
            }

            //Set the hours of service
            arrive_time = arrive_time + Graph[route.R.get(route.R.size() - 2)][route.R.get(route.R.size() - 1)].duration
                    + AllCustomers.get(route.R.get(route.R.size() - 2)).serve/60;
            time = arrive_time - route.startTime;
            if ((time - timelimit) > epsl){ //Check hours of service limits
                return false;
            }
        }

        //Verify the calculation for routes
        if (check && route.R.size() > 4) {
            if (Math.abs(route.cost - cost) > epsl) {
                System.err.printf("####### Route cost NOT consistent: %.4f, %.4f ####### %n", route.cost, cost);
                return false;
            }

            if (Math.abs(route.load - load) > epsl) {
                System.err.printf("####### Route load NOT consistent: %.4f, %.4f ####### %n", route.load, load);
                return false;
            }
            //Otherwise set the route information
        } else {
            route.dist = dist;
            route.cost = cost;
            route.time = time;
            route.subt = subt;
            route.load = load;
        }
        return true;
    }


    public boolean checkRoute(Route route){
        int n = route.R.size();
        double load = 0, time = 0, dist = 0, cost = 0, epsl = 1e-5;

        //Set the distance
        for (int i = 1; i < route.R.size() - 2; i++) {
            dist += Graph[route.R.get(i)][route.R.get(i + 1)].distance;
        }

        //Set the cost
        if (route.R.size() == 4) { //LTL route
            cost = SignleCost[route.R.get(1)];
            route.cost = cost;
            return true;

        } else { //MSTL route
            cost = Graph[route.R.get(1)][route.R.get(route.R.size()-2)].distance *
                    Rate[route.R.get(1)][route.R.get(route.R.size()-2)] + (route.R.size()-4) * stop
                    + (dist-Graph[route.R.get(1)][route.R.get(route.R.size()-2)].distance) * detour;
            route.cost = cost;
        }

        //MSTL route: check depots
        if (route.R.get(0) !=0 || route.R.get(n-1) != 2*NUM + 1) {
            System.err.printf("###### Error route in depots: %d, %d ###### %n", route.R.get(0), route.R.get(n-1));
            return false;
        }

        //MSTL route: bundling and time constraints
        if (n > 2*maxOrders + 2) { //bundling size limit
            int b = (n-2)/2;
            System.err.printf("###### Error route in bundling size: %d, maxOrder %d ###### %n",
                    b, maxOrders);
            return false;
        }

        for (int i = 1; i <= n/2 - 2; i++) { //bundling radius constraints for pickup
            if (Graph[route.R.get(i)][route.R.get(i + 1)].distance > radius) {
                double dis = Graph[route.R.get(i)][route.R.get(i + 1)].distance;
                System.err.printf("###### Error route in pickup bundling radius: %.2f, %.2f ###### %n",
                        dis, radius);
                return false;
            }
        }
        for (int i = n/2; i <= n - 3; i++) { //bundling radius constraints for delivery
            if (Graph[route.R.get(i)][route.R.get(i + 1)].distance > radius) {
                double dis = Graph[route.R.get(i)][route.R.get(i + 1)].distance;
                System.err.printf("###### Error route in delivery bundling radius: %.2f, %.2f ###### %n",
                        dis, radius);
                return false;
            }
        }

        // Set the load
        for(int i = 1; i < route.R.size()/2; i++){
            load = load + AllCustomers.get(route.R.get(i)).demand;
        }
        if (load > capacity) {
            System.err.printf("###### Error route in load capacity: %.2f, %.2f ###### %n",
                    load, capacity);
            return false;
        }

        //Time conditions
        double st = AllCustomers.get(route.R.get(1)).early - Graph[0][AllCustomers.get(route.R.get(1)).id].duration;
        route.setStartTime(st);
        //路径的出发时间自动定位为第一个订单的出发时间
        double arrive_time = route.startTime;

        //pickup arrival time
        for (int i = 1; i < route.R.size() / 2; i++) {
            int current = route.R.get(i), before = route.R.get(i - 1);
            arrive_time = arrive_time + Graph[before][current].duration + AllCustomers.get(before).serve/60;
            Customer customer = AllCustomers.get(current);
            if (arrive_time <= customer.early) {
                arrive_time = customer.early;
            }
        }

        //delivery arrival time
        for (int i = route.R.size() / 2; i < route.R.size() - 1; i++) {
            int current = route.R.get(i), before = route.R.get(i - 1);
            arrive_time = arrive_time + Graph[before][current].duration + AllCustomers.get(before).serve/60;
            Customer customer = AllCustomers.get(current);
            if (arrive_time > customer.last) {
                System.out.printf("####### Infeasible route violate delivery time of order %d: ! ####### %n",
                        current);
                return false;
            }
        }

        //Set the hours of service
        arrive_time = arrive_time + Graph[route.R.get(route.R.size() - 2)][route.R.get(route.R.size() - 1)].duration
                + AllCustomers.get(route.R.get(route.R.size() - 2)).serve/60;
        time = arrive_time - route.startTime;
        if ((time - timelimit) > epsl){ //Check hours of service limits
            System.out.printf("####### Infeasible route violate hours of service limit %.2f: %.2f! #######%n",
                    timelimit, time);
            return false;
        }

        return true;
    }


    // 结果记录部分参数初始化
    public void ParamsInitial() {
        //Eaxct algorithm
        numVar = 0;
        numConstr = 0;
        LTL_exact = 0;
        MSTL_exact_2 = 0;
        MSTL_exact_3 = 0;
        MSTL_exact_4 = 0;
        MSTL_exact_5M = 0;
        feasb_exact = 0;
        COST_exact = 0;
        TIME_exact = 0;
        gap_exact = 0;

        //Enumeration algorithm
        EnumNum = 0;
        Two_route = 0;
        Three_route = 0;
        Four_route = 0;
        Five_route = 0;

        LTL_enum = 0;
        MSTL_enum_2 = 0;
        MSTL_enum_3 = 0;
        MSTL_enum_4 = 0;
        MSTL_enum_5M = 0;
        COST_enum = 0;
        ITER_enum = 0;
        TIME_enum = 0;

        // 2-OPT algorithm
        OPTNum = 0;
        LTL_Opt = 0;
        MSTL_Opt_2 = 0;
        MSTL_Opt_3 = 0;
        MSTL_Opt_4 = 0;
        MSTL_Opt_5M = 0;
        COST_Opt = 0;
        TIME_Opt = 0;
        Swap_Opt = 0;
        ITER_Opt = 0;

        // Column generation algorithm
        RCGNum = 0;
        RCG_2 = 0;
        RCG_3 = 0;
        RCG_4 = 0;
        RCG_5M = 0;

        LTL_label = 0;
        MSTL_label_2 = 0;
        MSTL_label_3 = 0;
        MSTL_label_4 = 0;
        MSTL_label_5M = 0;
        COST_label = 0;
        ITER_label = 0;
        TIME_label = 0;
        impr_label = 0;
        gap_label = 0;
        gap_LP_label = 0;
        TIME_LP_label = 0;
        TIME_SP_label = 0;
        TIME_IP_label = 0;
        CG_label = 0;
    }

    public String makeCsvItem() {
        String str;
        str = inst_name + "," + netCase + "," + NUM + "," + numV + "," + numE + ","
                + LIFO + "," + maxOrders + "," + radius + "," + stop + "," + detour + ","
                + OriginCost + "," + numVar + "," + numConstr + ","
                + LTL_exact + "," + MSTL_exact_2 + "," + MSTL_exact_3 + "," + MSTL_exact_4 + "," + MSTL_exact_5M + ","
                + feasb_exact + "," + COST_exact + "," + TIME_exact + "," + gap_exact + ","
                + EnumNum + "," + Two_route + "," + Three_route + "," + Four_route + "," + Five_route + ","
                + LTL_enum + "," + MSTL_enum_2 + "," + MSTL_enum_3 + "," + MSTL_enum_4 + "," + MSTL_enum_5M + ","
                + COST_enum + "," + ITER_enum + "," + TIME_enum + ","
                + OPTNum + "," + LTL_Opt + "," + MSTL_Opt_2 + "," + MSTL_Opt_3 + "," + MSTL_Opt_4 + "," + MSTL_Opt_5M + ","
                + COST_Opt + "," + TIME_Opt + "," + Swap_Opt + "," + ITER_Opt + ","
                + RCGNum + "," + RCG_2 + "," + RCG_3 + "," + RCG_4 + "," + RCG_5M + ","
                + LTL_label + "," + MSTL_label_2 + "," + MSTL_label_3 + "," + MSTL_label_4 + "," + MSTL_label_5M + ","
                + COST_label + "," + ITER_label + "," + TIME_label + "," + impr_label + "," + gap_label + ","
                + gap_LP_label + "," + TIME_LP_label + "," + TIME_SP_label + "," + TIME_IP_label + "," + CG_label;
        return str;
    }

    // 路径和成本分配结果写入json文件
    /*public String resultToString(){
        StringBuilder sb = new StringBuilder();

        // results of enumeration
        if (!large) {
            sb.append("\"COST_label\": \r\n");
            sb.append(COST_label + "\r\n");

            for (int i : routesLabel.keySet()) {
                sb.append(i + ":" + Arrays.toString(routesLabel.get(i).R.toArray()) + "\r\n");
            }

            sb.append("\r\n");
        }

        return sb.toString();
    }*/


}
