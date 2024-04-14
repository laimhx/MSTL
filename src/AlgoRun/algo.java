package AlgoRun;

import Common.*;

import PickandDelivery.PDP2_opt.PDP2opt;
import PickandDelivery.PDPLabel.PDP_label;
import PickandDelivery.PDPEaxct.PDP_exact;
import PickandDelivery.PDPEaxct.PDP_enmu;

import java.text.ParseException;
import java.util.HashMap;
import java.util.HashSet;

import static AlgoRun.algoParam.Exper;

public class algo {
    public Instance instance;  // 求解实例
    public double gap_Opt;
    public double impr_Opt;
    public double impr_enum;
    public double impr_exact;


    public algo(Instance instance) {
        this.instance = instance;
    }


    public void run( ) throws ParseException {
        System.out.printf("******** Start test %s: %d orders, %b LIFO, %d limit, %.2fkm radius, %.2f$ stop-off, %.2f$ detour ********%n",
                instance.inst_name, instance.NUM, instance.LIFO, instance.maxOrders, instance.radius, instance.stop, instance.detour);

        //initial gaps and improvements
        impr_exact = -999.0; impr_enum = -999.0;
        gap_Opt = -999.0; instance.gap_label = -999.0;

        //2-Opt algorithm
        routeSolveOpt();

        //column generation algorithm
        routeSolveLabel();

        //enumeration algorithm
        if(instance.NUM < 1000) {
            routeSolveEnum();
        }

        //exact algorithm
        if(instance.NUM < 100) {
            routeSolveExact();
        }

        //the final gaps
        if(instance.NUM < 1000) {
            gap_Opt = 100 * (instance.COST_Opt - instance.COST_enum) / instance.COST_enum;
            instance.gap_label = 100 * (instance.COST_label - instance.COST_enum) / instance.COST_enum;
            instance.gap_exact = 100 * (instance.COST_exact - instance.COST_enum) / instance.COST_enum;
        }

        // Summary results
        System.out.println("*********************************************************");
        System.out.println("Optimization results: (LTL, impr_exact, impr_enum, impr_opt, impr_label) = ");
        System.out.printf("(%.2f, %.4f%%, %.4f%%, %.4f%%, %.4f%%) %n", instance.OriginCost, impr_exact, impr_enum,
                impr_Opt, instance.impr_label);
        System.out.printf("Optimality gaps: (enum, gap_exact, gap_opt, gap_label) = (%.2f, %.4f%%, %.4f%%, %.4f%%) %n",
                instance.COST_enum, instance.gap_exact, gap_Opt, instance.gap_label);
        System.out.printf("Complexity: exact %.4f min, enum %.4f min, Opt %.4f min, CG %.4f min %n",
                instance.TIME_exact, instance.TIME_enum, instance.TIME_Opt, instance.TIME_label);
        System.out.printf("Routes in the CG solution: 1-,2-,3-,4-,5+: %d, %d, %d, %d, %d %n", instance.LTL_label,
                instance.MSTL_label_2, instance.MSTL_label_3, instance.MSTL_label_4, instance.MSTL_label_5M);
        System.out.printf("MSTL Routes generated for %d orders: enum %d, opt %d, CG %d %n",
                instance.NUM, instance.EnumNum, instance.OPTNum, instance.RCGNum);
        System.out.printf("Parameters: %b LIFO, %d limit, %.2fkm radius, %.2f$ stop-off, %.2f$ detour %n",
                instance.LIFO, instance.maxOrders, instance.radius, instance.stop, instance.detour);
    }


    // Exact algorithm by Gurobi
    public void routeSolveExact() throws ParseException {
        System.out.println("------------------- Start exact algorithm ------------------");
        String sol = "Exact routes";
        PDP_exact pdp_exact = new PDP_exact(instance);
        pdp_exact.run();

        /*
        SolutionModify(instance.routesExact, sol);
        CheckRoutes(instance.routesExact, sol);
        */
        instance.COST_exact = Calculation(instance.routesExact);
        impr_exact = 100 * (instance.OriginCost-instance.COST_exact)/instance.OriginCost;

        // Check route use
        instance.LTL_exact = 0;
        instance.MSTL_exact_2 = 0;
        instance.MSTL_exact_3 = 0;
        instance.MSTL_exact_4 = 0;
        instance.MSTL_exact_5M = 0;
        for(Route route:instance.routesExact.values()){
            if (route.R.size() == 4) {
                instance.LTL_exact ++;
            } else {
                if (route.R.size() == 6) {
                    instance.MSTL_exact_2 ++;
                } else if (route.R.size() == 8) {
                    instance.MSTL_exact_3 ++;
                } else if (route.R.size() == 10) {
                    instance.MSTL_exact_4 ++;
                } else if (route.R.size() >= 12) {
                    instance.MSTL_exact_5M ++;
                }
            }
        }

        // Results of exact algorithm
        System.out.printf("Exact optimization results: (LTL, exact, impr) = (%.2f, %.2f, %.4f%%) %n",
                instance.OriginCost, instance.COST_exact, impr_exact);
        System.out.printf("Routes in the exact solution: 1-,2-,3-,4-,5M: %d, %d, %d, %d, %d %n",
                instance.LTL_exact, instance.MSTL_exact_2, instance.MSTL_exact_3, instance.MSTL_exact_4,
                instance.MSTL_exact_5M);
    }


    // Exact algorithm by enumeration
    public void routeSolveEnum() throws ParseException {
        System.out.println("------------------- Start enumeration algorithm ------------------");
        String sol = "Enum routes";
        PDP_enmu pdp_enmu = new PDP_enmu(instance);
        pdp_enmu.run();

        SolutionModify(instance.routesEnmu, sol);
        CheckRoutes(instance.routesEnmu, sol);
        instance.COST_enum = Calculation(instance.routesEnmu);
        impr_enum = 100 * (instance.OriginCost-instance.COST_enum)/instance.OriginCost;

        // Check route use
        instance.LTL_enum = 0;
        instance.MSTL_enum_2 = 0;
        instance.MSTL_enum_3 = 0;
        instance.MSTL_enum_4 = 0;
        instance.MSTL_enum_5M = 0;
        for (Route route:instance.routesEnmu.values()) {
            if (route.R.size() == 4) {
                instance.LTL_enum ++;
            } else {
                if (route.R.size() == 6) {
                    instance.MSTL_enum_2 ++;
                } else if (route.R.size() == 8) {
                    instance.MSTL_enum_3 ++;
                } else if (route.R.size() == 10) {
                    instance.MSTL_enum_4 ++;
                } else if (route.R.size() >= 12) {
                    instance.MSTL_enum_5M ++;
                }
            }
        }

        // Results of enumeration algorithm
        System.out.printf("Enumeration optimization results: (LTL, enum, impr) = (%.2f, %.2f, %.4f%%) %n",
                instance.OriginCost, instance.COST_enum, impr_enum);
        System.out.printf("Routes in the enumeration solution: 1-,2-,3-,4-,5-: %d, %d, %d, %d, %d %n",
                instance.LTL_enum, instance.MSTL_enum_2, instance.MSTL_enum_3, instance.MSTL_enum_4,
                instance.MSTL_enum_5M);
    }


    //2-Opt heuristic algorithm
    public void routeSolveOpt() throws ParseException {
        System.out.println("------------------- Start 2-Opt heuristic algorithm -------------------");
        String sol = "Opt routes";
        PDP2opt pdp2opt = new PDP2opt(instance);
        pdp2opt.run();

        SolutionModify(instance.routesOpt, sol);
        CheckRoutes(instance.routesOpt, sol);
        instance.COST_Opt = Calculation(instance.routesOpt);
        impr_Opt = 100*(instance.OriginCost - instance.COST_Opt)/instance.OriginCost;

        // Check route use
        instance.LTL_Opt = 0;
        instance.MSTL_Opt_2 = 0;
        instance.MSTL_Opt_3 = 0;
        instance.MSTL_Opt_4 = 0;
        instance.MSTL_Opt_5M = 0;
        for(Route route: instance.routesOpt.values()){
            if (route.R.size() == 4) {
                instance.LTL_Opt ++;
            } else {
                if (route.R.size() == 6) {
                    instance.MSTL_Opt_2 ++;
                } else if (route.R.size() == 8) {
                    instance.MSTL_Opt_3 ++;
                } else if (route.R.size() == 10) {
                    instance.MSTL_Opt_4 ++;
                } else if (route.R.size() >= 12) {
                    instance.MSTL_Opt_5M ++;
                }
            }
        }

        // Results of 2-Opt
        System.out.printf("Algorithm 2-Opt results: (LTL, OPT, impr) = (%.2f, %.2f, %.4f%%) %n",
                instance.OriginCost, instance.COST_Opt, impr_Opt);
        System.out.printf("Routes in the 2-Opt solution: 1-,2-,3-,4-,5M: %d, %d, %d, %d, %d %n",
                instance.LTL_Opt, instance.MSTL_Opt_2, instance.MSTL_Opt_3, instance.MSTL_Opt_4,
                instance.MSTL_Opt_5M);
    }


    //Column generation algorithm
    public void routeSolveLabel(){
        System.out.println("------------------- Start column generation algorithm -------------------");
        String sol = "CG routes";
        PDP_label pdp_label = new PDP_label(instance);
        pdp_label.run();

        SolutionModify(instance.routesLabel, sol);
        CheckRoutes(instance.routesLabel, sol);  // 检查成本计算
        instance.COST_label = Calculation(instance.routesLabel);
        instance.impr_label = 100*(instance.OriginCost - instance.COST_label)/instance.OriginCost;

        // Check route use
        instance.LTL_label = 0;
        instance.MSTL_label_2 = 0;
        instance.MSTL_label_3 = 0;
        instance.MSTL_label_4 = 0;
        instance.MSTL_label_5M = 0;
        for(Route route: instance.routesLabel.values()){
            if (route.R.size() == 4) {
                instance.LTL_label ++;
            } else {
                if (route.R.size() == 6) {
                    instance.MSTL_label_2 ++;
                } else if (route.R.size() == 8) {
                    instance.MSTL_label_3 ++;
                } else if (route.R.size() == 10) {
                    instance.MSTL_label_4 ++;
                } else if(route.R.size() >= 12) {
                    instance.MSTL_label_5M ++;
                }
            }
        }

        // Results of CG
        System.out.printf("Algorithm CG results: (LTL, CG, impr) = (%.2f, %.2f, %.4f%%) %n",
                instance.OriginCost, instance.COST_label, instance.impr_label);
        System.out.printf("Routes in the CG solution: 1-,2-,3-,4-,5M: %d, %d, %d, %d, %d %n",
                instance.LTL_label, instance.MSTL_label_2, instance.MSTL_label_3, instance.MSTL_label_4,
                instance.MSTL_label_5M);
    }


    //Double-check the validity of solution
    public void CheckRoutes(HashMap<Integer, Route> Routes, String sol){
        //所有顾客都被访问且只被访问一次
        HashSet<Integer> customers = new HashSet<>();
        for (Route route : Routes.values()) {
            boolean isValid = (instance.bundle_feasible(route) & instance.feasible(route,true));
            if (!isValid) {
                System.err.printf("####### Warning: Calculation error in Route ID. %d of size %d from %s! ####### %n",
                        route.id, route.R.size(), sol);
                System.err.printf("Bundling feasibility: %b %n", instance.bundle_feasible(route));
            }
            for (int i = 1; i < route.R.size()/2; i++) {
                customers.add(route.R.get(i));
            }
        }

        if (customers.size() != instance.NUM) {
            System.err.printf("####### Warning: orders are NOT all served (%d, %d) for %s! ####### %n",
                    customers.size(), instance.NUM, sol);
        }
    }


    //Compute the total cost of routes
    public double Calculation(HashMap<Integer,Route> Routes) {
        double Costs = 0;
        for(Route route : Routes.values()){
            Costs += route.cost;
        }
        return Costs;
    }


    //Revise the solution to include missed orders
    public void SolutionModify(HashMap<Integer, Route> Routes, String sol){
        //record the served orders
        HashSet<Integer> customers = new HashSet<>();
        for (Route route : Routes.values()) {
            for (int i = 1; i < route.R.size()/2; i++) {
                customers.add(route.R.get(i));
            }
        }

        int dummy_id = -1;
        for (int i = 1; i <= instance.NUM; i++) {
            if (!customers.contains(i)) {
                System.out.printf("###### Repair the %s for missed order ID %d ###### %n", sol, i);
                Route route = new Route();
                route.R.add(0);
                route.R.add(i);
                route.R.add(i+instance.NUM);
                route.R.add(2*instance.NUM+1);
                route.cost = instance.SignleCost[i];
                route.id = dummy_id;
                dummy_id --;
                Routes.put(route.id,route);
            }
        }
    }



}
