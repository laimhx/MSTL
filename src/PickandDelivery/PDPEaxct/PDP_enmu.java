package PickandDelivery.PDPEaxct;

import Common.*;

import java.text.ParseException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;

//import gurobi.*;
import com.gurobi.gurobi.*;

import static AlgoRun.algoParam.capacity;


public class PDP_enmu {
    public Instance instance;
    public ArrayList<Route> AllRoutes;
    public int NUM;
    public int num_LTL;
    public int num_MSTL2;
    public int num_MSTL3;
    public int num_MSTL4;
    public int num_MSTL5;
    public double iters;
    public double maxTime;
    public double eps;


    public PDP_enmu(Instance instance) {
        this.instance = instance;
        this.NUM = instance.NUM;
    }


    public void run() throws ParseException {
        long startTime = System.currentTimeMillis();
        maxTime = 240.0; //4 hours
        eps = 1e-5;

        //start the algorithm
        enmuRoutes();
        System.out.printf("Solving the set partitioning problem with orders %d, routes %d %n",
                NUM, AllRoutes.size());
        instance.routesEnmu = solveMP();

        //Algorithm stopped
        long endTime = System.currentTimeMillis();
        double tt = (endTime - startTime)/1000.0/60;
        System.out.printf("------- Enumeration algorithm is completed with %.4f min: %d 2-routes, %d 3-routes, " +
                        "%d 4-routes, %d 5-routes -------%n", tt, num_MSTL2, num_MSTL3, num_MSTL4, num_MSTL5);

        //Record the results
        instance.Two_route = num_MSTL2;
        instance.Three_route = num_MSTL3;
        instance.Four_route = num_MSTL4;
        instance.Five_route = num_MSTL5;
        instance.EnumNum = num_MSTL2 + num_MSTL3 + num_MSTL4 + num_MSTL5;
        instance.ITER_enum = iters/(10^2);
        instance.TIME_enum = tt;

        //clear temp data
        AllRoutes.clear();
    }


    //Enumerate all instance.feasible routes
    public void enmuRoutes() throws ParseException {
        AllRoutes = new ArrayList<>();
        iters = 0;

        //枚举所有可行的单独路径
        int startID = 0, id = 0;
        for (int i = 1; i <= NUM; i++) {
            Route route = new Route();
            route.id = id;
            route.R.add(0);
            route.R.add(i);
            route.R.add(i + NUM);
            route.R.add(2 * NUM + 1);
            route.cost = instance.SignleCost[i];

            AllRoutes.add(route);
            id ++;
        }
        num_LTL = NUM;
        iters += (double) (NUM /10^6);

        // 所有可行的包含两个订单的路径
        startID = num_LTL;
        ArrayList<Route> Two_routes;
        if (instance.LIFO) {
            Two_routes = ENUM_TwoRoutes_LIFO(startID);
        } else {
            Two_routes = ENUM_TwoRoutes(startID);
        }

        num_MSTL2 = Two_routes.size();
        AllRoutes.addAll(Two_routes);

        // 枚举所有可行包含3个订单的路径(最优路径）
        num_MSTL3 = 0;
        if (instance.maxOrders >= 3) {
            ArrayList<Route> Three_routes;
            startID = num_MSTL2 + num_LTL;
            if (instance.LIFO) {
                Three_routes = ENUM_ThreeRoutes_LIFO(startID);
            } else {
                Three_routes = ENUM_ThreeRoutes(startID);
            }

            num_MSTL3 = Three_routes.size();
            AllRoutes.addAll(Three_routes);
        }

        // 枚举所有可行包含4个订单的路径(最优路径）
        num_MSTL4 = 0;
        if (instance.maxOrders >= 4) {
            ArrayList<Route> Four_routes;
            startID = num_MSTL3 + num_MSTL2 + num_LTL;
            if (instance.LIFO) {
                Four_routes = ENUM_FourRoutes_LIFO(startID);
            } else {
                Four_routes = ENUM_FourRoutes(startID);
            }

            num_MSTL4 = Four_routes.size();
            AllRoutes.addAll(Four_routes);
        }

        // 枚举所有可行包含5个订单的路径(最优路径）
        num_MSTL5 = 0;
        if (instance.maxOrders >= 5) {
            ArrayList<Route> Five_routes;
            startID = num_MSTL3 + num_MSTL2 + num_LTL;
            if (instance.LIFO) {
                Five_routes = ENUM_FiveRoutes_LIFO(startID);
            } else {
                Five_routes = ENUM_FiveRoutes(startID);
            }

            num_MSTL5 = Five_routes.size();
            AllRoutes.addAll(Five_routes);
        }

        //Also add CG routes if possible
        int CGn = 0;
        for (Route route : instance.CGRoutes) {
            if (isNewRoute(AllRoutes,route)) {
                AllRoutes.add(route);
                CGn ++;

                if (route.R.size() == 6) {
                    num_MSTL2 ++;
                } else if (route.R.size() == 8) {
                    num_MSTL3 ++;
                } else if (route.R.size() == 10) {
                    num_MSTL4 ++;
                } else if (route.R.size() >= 12) {
                    num_MSTL5 ++;
                }
            }
        }

        System.out.printf("###### Add %d CG routes into enum Routes. ###### %n", CGn);
    }


    public boolean isNewRoute(ArrayList<Route> Routes, Route route) {
        for (Route r : Routes) {
            if (r.R.size() == route.R.size() && Math.abs(r.load - route.load) < eps &&
                    Math.abs(r.cost - route.cost) < eps) {
                return false; //already included
            }
        }
        return true; //a new route
    }


    // Gurobi solver
    public HashMap<Integer, Route> solveMP() {
        HashMap<Integer,Route> routeSol = new HashMap<>();
        try {
            //Set up the model
            GRBEnv env = new GRBEnv();
            GRBModel model = new GRBModel(env);
            model.set(GRB.StringAttr.ModelName, "enumModel");
            GRBLinExpr expr;

            //Process data: constraint matrix
            double[][] Constr = new double[NUM + 1][AllRoutes.size()]; //(NUM+1) x R
            for (int j = 0; j < AllRoutes.size(); j++) {
                Route route = AllRoutes.get(j);
                for (int k = 1; k < route.R.size() / 2; k++) {
                    int i = route.R.get(k);
                    Constr[i][j] = 1;
                }
            }

            //Set variables
            GRBVar[] x = new GRBVar[AllRoutes.size()];
            for (int j = 0; j < AllRoutes.size(); j++) {
                x[j] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x" + "_" + j);
            }

            //Set objective function
            expr = new GRBLinExpr();
            for (int j = 0; j < AllRoutes.size(); j++) {
                Route r = AllRoutes.get(j);
                expr.addTerm(r.cost, x[j]);
            }
            model.setObjective(expr, GRB.MINIMIZE);

            //Set-covering constraints: sum_{r}a_{ir}x_{r} = 1;
            for (int i = 1; i <= NUM; i++) {
                expr = new GRBLinExpr();
                expr.addTerms(Constr[i], x);
                // model.addConstr(expr, GRB.GREATER_EQUAL, 1.0, "cons" + i);
                model.addConstr(expr, GRB.EQUAL, 1.0, "cons" + i);
            }

            //The optimization parameters
            model.set(GRB.IntParam.OutputFlag, 0); //no log output
            model.set(GRB.DoubleParam.TimeLimit, 1800); //time limit: 30 min

            //Solve the model
            model.optimize();

            //Check validity of solution
            int status = model.get (GRB.IntAttr.Status);
            if (status == GRB.Status.INF_OR_UNBD || status == GRB.Status.INFEASIBLE){
                System.out.printf("###### The enumeration problem was stopped with status %d ###### %n", status);
                System.exit(0);
            }

            //Determine the selected routes
            for (int j = 0; j < AllRoutes.size(); j++) {
                if (Math.abs(x[j].get(GRB.DoubleAttr.X) - 1) <= eps) {
                    routeSol.put(AllRoutes.get(j).id, AllRoutes.get(j));
                }
            }

            //Clear the model
            model.dispose();
            env.dispose();

        } catch (GRBException e) {
            System.err.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
        }

        //result
        return routeSol;
    }


    public ArrayList<Route> ENUM_TwoRoutes(int startId){
        int id = 0, coaln = 0, kr = 0;
        long iter = 0;
        double best, tt;
        HashSet<Integer> coalKey;
        int[] coalition;

        HashMap<HashSet<Integer>, Integer> Coalitions = new HashMap<>();
        ArrayList<Route> allRoutes = new ArrayList<>();
        ArrayList<Route> Routes = new ArrayList<>();
        Route route, best_route;

        long startTime = System.currentTimeMillis();
        for (int i = 1; i <= NUM; i++) {
            for (int j = 1; j <= NUM; j++) {
                if (i == j) {
                    continue;
                }
                //容量约束
                if (instance.AllCustomers.get(i).demand + instance.AllCustomers.get(j).demand > capacity) {
                    continue;
                }

                //the best route for coalition
                coalition = new int[]{i, j};
                coaln ++;
                best = Double.MAX_VALUE;
                best_route = new Route();

                for (int pick1 : coalition) {
                    for (int pick2 : coalition) {
                        if (pick1 == pick2 || instance.Graph[pick1][pick2].distance > instance.radius) {
                            continue; // 两个pick点之间满足半径约束
                        }
                        for (int deli1 : coalition) {
                            for (int deli2 : coalition) {
                                if (deli1 == deli2 || instance.Graph[deli1+NUM][deli2+NUM].distance > instance.radius) {
                                    continue;
                                }

                                route = new Route();
                                route.R.add(0);
                                route.R.add(pick1);
                                route.R.add(pick2);
                                route.R.add(deli1 + NUM);
                                route.R.add(deli2 + NUM);
                                route.R.add(2 * NUM + 1);

                                //select the best route
                                if (instance.feasible(route,false) && route.cost < best) {
                                    best = route.cost;
                                    best_route = route.cloneRoute();
                                }
                                iter ++;
                            }
                        }
                    }
                }

                //Check economic validity
                double LTLs = instance.SignleCost[i] + instance.SignleCost[j];
                if (best != Double.MAX_VALUE && best < LTLs) {
                    best_route.id = id;
                    allRoutes.add(best_route);

                    //store the valid coalition
                    coalKey = new HashSet<>();
                    coalKey.add(i); coalKey.add(j);
                    if (Coalitions.get(coalKey) == null) {
                        Coalitions.put(coalKey, id);
                    } else {
                        kr = Coalitions.get(coalKey); //route id
                        Route r = allRoutes.get(kr); //previous optimum route
                        if (best < r.cost) {
                            Coalitions.replace(coalKey, id);
                        }
                    }
                    id ++;
                }
            }
        }

        // Select the optimal routes
        System.out.println("Determine the optimal routes for 2-coalitions ...");
        id = startId;
        for (HashSet<Integer> key : Coalitions.keySet()) {
            kr = Coalitions.get(key);
            Route r = allRoutes.get(kr);
            r.id = id;
            Routes.add(r);
            id ++;
        }
        allRoutes.clear();
        Coalitions.clear();

        //end
        iters += (double) iter/(10^6);
        tt = (System.currentTimeMillis() - startTime)/1000.0/60;
        System.out.printf("Enumerating 2-routes stopped: coal %d, iter %d, time %.2f min, routes %d %n",
                coaln, iter, tt, Routes.size());
        return Routes;
    }


    public ArrayList<Route> ENUM_TwoRoutes_LIFO(int startId) {
        int id = 0, coaln = 0, kr = 0;
        long iter = 0;
        double best, tt;
        HashSet<Integer> coalKey;
        int[] coalition;

        HashMap<HashSet<Integer>, Integer> Coalitions = new HashMap<>();
        ArrayList<Route> allRoutes = new ArrayList<>();
        ArrayList<Route> Routes = new ArrayList<>();
        Route route, best_route;

        long startTime = System.currentTimeMillis();
        for (int i = 1; i <= NUM; i++) {
            for (int j = 1; j <= NUM; j++) {
                if (i == j) {
                    continue;
                }
                //容量约束
                if (instance.AllCustomers.get(i).demand + instance.AllCustomers.get(j).demand > capacity) {
                    continue;
                }

                //the best route for coalition
                coalition = new int[]{i, j};
                coaln ++;
                best = Double.MAX_VALUE;
                best_route = new Route();

                for (int pick1 : coalition) {
                    for (int pick2 : coalition) {
                        if (pick1 == pick2 || instance.Graph[pick1][pick2].distance > instance.radius) {
                            continue; //两个pick点之间满足半径约束
                        }
                        //LIFO
                        if(instance.Graph[pick2+NUM][pick1+NUM].distance > instance.radius) {
                            continue;
                        }

                        route = new Route();
                        route.R.add(0);
                        route.R.add(pick1);
                        route.R.add(pick2);
                        route.R.add(pick2 + NUM);
                        route.R.add(pick1 + NUM);
                        route.R.add(2 * NUM + 1);

                        //select the best route
                        if (instance.feasible(route, false) && route.cost < best) {
                            best = route.cost;
                            best_route = route.cloneRoute();
                        }
                        iter ++;
                    }
                }

                //Check economic validity
                double LTLs = instance.SignleCost[i] + instance.SignleCost[j];
                if (best != Double.MAX_VALUE && best < LTLs) {
                    best_route.id = id;
                    allRoutes.add(best_route);

                    //store the valid coalition
                    coalKey = new HashSet<>();
                    coalKey.add(i); coalKey.add(j);
                    if (Coalitions.get(coalKey) == null) {
                        Coalitions.put(coalKey, id);
                    } else {
                        kr = Coalitions.get(coalKey); //route id
                        Route r = allRoutes.get(kr); //previous optimum route
                        if (best < r.cost) {
                            Coalitions.replace(coalKey, id);
                        }
                    }
                    id ++;
                }
            }
        }

        // Select the optimal routes
        System.out.println("Determine the optimal routes for 2-coalitions ...");
        id = startId;
        for (HashSet<Integer> key : Coalitions.keySet()) {
            kr = Coalitions.get(key);
            Route r = allRoutes.get(kr);
            r.id = id;
            Routes.add(r);
            id ++;
        }
        allRoutes.clear();
        Coalitions.clear();

        //end
        iters += (double) iter/(10^6);
        tt = (System.currentTimeMillis() - startTime)/1000.0/60;
        System.out.printf("Enumerating LIFO 2-routes stopped: coal %d, iter %d, time %.2f min, routes %d %n",
                coaln, iter, tt, Routes.size());
        return Routes;
    }


    public ArrayList<Route> ENUM_ThreeRoutes(int startId) {
        int id = 0, coaln = 0, kr = 0;
        long iter = 0;
        double best, tt;
        HashSet<Integer> coalKey;
        int[] coalition;

        HashMap<HashSet<Integer>, Integer> Coalitions = new HashMap<>();
        ArrayList<Route> allRoutes = new ArrayList<>();
        ArrayList<Route> Routes = new ArrayList<>();
        Route route, best_route;

        long startTime = System.currentTimeMillis();
        for (int i = 1; i <= NUM; i++) {
            for (int j = 1; j <= NUM; j++) {
                if (i == j) {
                    continue;
                }
                for (int k = 1; k <= NUM; k++) {
                    if (k == j || k == i) {
                        continue;
                    }
                    //装载约束
                    if (instance.AllCustomers.get(i).demand + instance.AllCustomers.get(j).demand +
                            instance.AllCustomers.get(k).demand > capacity) {
                        continue;
                    }

                    //the best route for coalition
                    coalition = new int[]{i, j, k};
                    coaln ++;
                    best = Double.MAX_VALUE;
                    best_route = new Route();

                    for (int pick1 : coalition) {
                        for (int pick2 : coalition) {
                            if (pick1 == pick2 || instance.Graph[pick1][pick2].distance > instance.radius) {
                                continue;
                            }
                            for (int pick3 : coalition) {
                                if (pick1 == pick3 || pick2 == pick3 || instance.Graph[pick2][pick3].distance
                                        > instance.radius) {
                                    continue;
                                }
                                for (int deli1 : coalition) {
                                    for (int deli2 : coalition) {
                                        if (deli1 == deli2 || instance.Graph[deli1 + NUM][deli2 + NUM].distance
                                                > instance.radius) {
                                            continue;
                                        }
                                        for (int deli3 : coalition) {
                                            if (deli1 == deli3 || deli2 == deli3 || instance.Graph[deli2 + NUM][deli3 + NUM].distance
                                                    > instance.radius) {
                                                continue;
                                            }

                                            route = new Route();
                                            route.R.add(0);
                                            route.R.add(pick1);
                                            route.R.add(pick2);
                                            route.R.add(pick3);
                                            route.R.add(deli1 + NUM);
                                            route.R.add(deli2 + NUM);
                                            route.R.add(deli3 + NUM);
                                            route.R.add(2 * NUM + 1);

                                            //select the best route
                                            if (instance.feasible(route,false) && route.cost < best) {
                                                best = route.cost;
                                                best_route = route.cloneRoute();
                                            }
                                            iter ++;
                                        }
                                    }
                                }
                            }
                        }
                    }

                    //Check economic validity
                    double LTLs = instance.SignleCost[i] + instance.SignleCost[j] + instance.SignleCost[k];
                    if (best != Double.MAX_VALUE && best < LTLs) {
                        best_route.id = id;
                        allRoutes.add(best_route);

                        //store the valid coalition
                        coalKey = new HashSet<>();
                        coalKey.add(i); coalKey.add(j); coalKey.add(k);
                        if (Coalitions.get(coalKey) == null) {
                            Coalitions.put(coalKey, id);
                        } else {
                            kr = Coalitions.get(coalKey); //route id
                            Route r = allRoutes.get(kr); //previous optimum route
                            if (best < r.cost) {
                                Coalitions.replace(coalKey, id);
                            }
                        }
                        id ++;
                    }
                }
            }
        }

        // Select the optimal routes
        System.out.println("Determine the optimal routes for 3-coalitions ...");
        id = startId;
        for (HashSet<Integer> key : Coalitions.keySet()) {
            kr = Coalitions.get(key);
            Route r = allRoutes.get(kr);
            r.id = id;
            Routes.add(r);
            id ++;
        }
        allRoutes.clear();
        Coalitions.clear();

        //end
        iters += (double) iter/(10^6);
        tt = (System.currentTimeMillis() - startTime)/1000.0/60;
        double cn = (double) coaln/10000;
        System.out.printf("Enumerating 3-routes stopped: coal %.2fe4, iter %d, time %.2f min, routes %d %n",
                cn, iter, tt, Routes.size());
        return Routes;
    }


    public ArrayList<Route> ENUM_ThreeRoutes_LIFO(int startId) {
        int id = 0, coaln = 0, kr = 0;
        long iter = 0;
        double best, tt;
        HashSet<Integer> coalKey;
        int[] coalition;

        HashMap<HashSet<Integer>, Integer> Coalitions = new HashMap<>();
        ArrayList<Route> allRoutes = new ArrayList<>();
        ArrayList<Route> Routes = new ArrayList<>();
        Route route, best_route;

        long startTime = System.currentTimeMillis();
        out:for (int i = 1; i <= NUM; i++) {
            for (int j = 1; j <= NUM; j++) {
                if (i == j) {
                    continue;
                }
                for (int k = 1; k <= NUM; k++) {
                    if (k == j || k == i) {
                        continue;
                    }
                    //装载约束
                    if (instance.AllCustomers.get(i).demand + instance.AllCustomers.get(j).demand +
                            instance.AllCustomers.get(k).demand > capacity) {
                        continue;
                    }

                    //the best route for coalition
                    coalition = new int[]{i, j, k};
                    coaln ++;
                    best = Double.MAX_VALUE;
                    best_route = new Route();

                    for (int pick1 : coalition) {
                        for (int pick2 : coalition) {
                            if (pick1 == pick2 || instance.Graph[pick1][pick2].distance > instance.radius) {
                                continue;
                            }
                            for (int pick3 : coalition) {
                                if (pick1 == pick3 || pick2 == pick3 || instance.Graph[pick2][pick3].distance
                                        > instance.radius) {
                                    continue;
                                }
                                //LIFO
                                if(instance.Graph[pick3+NUM][pick2+NUM].distance > instance.radius ||
                                        instance.Graph[pick2+NUM][pick1+NUM].distance > instance.radius) {
                                    continue;
                                }

                                route = new Route();
                                route.R.add(0);
                                route.R.add(pick1);
                                route.R.add(pick2);
                                route.R.add(pick3);
                                route.R.add(pick3 + NUM);
                                route.R.add(pick2 + NUM);
                                route.R.add(pick1 + NUM);
                                route.R.add(2 * NUM + 1);

                                //select the best route
                                if (instance.feasible(route, false) && route.cost < best) {
                                    best = route.cost;
                                    best_route = route.cloneRoute();
                                }
                                iter ++;
                            }
                        }
                    }

                    //Check economic validity
                    double LTLs = instance.SignleCost[i] + instance.SignleCost[j] + instance.SignleCost[k];
                    if (best != Double.MAX_VALUE && best < LTLs) {
                        best_route.id = id;
                        allRoutes.add(best_route);

                        //store the valid coalition
                        coalKey = new HashSet<>();
                        coalKey.add(i); coalKey.add(j); coalKey.add(k);
                        if (Coalitions.get(coalKey) == null) {
                            Coalitions.put(coalKey, id);
                        } else {
                            kr = Coalitions.get(coalKey); //route id
                            Route r = allRoutes.get(kr); //previous optimum route
                            if (best < r.cost) {
                                Coalitions.replace(coalKey, id);
                            }
                        }
                        id ++;
                    }

                    tt = (System.currentTimeMillis() - startTime)/1000.0/60;
                    if (tt > maxTime) {
                        break out; //exceeding the time limit
                    }
                }
            }
        }

        // Select the optimal routes
        System.out.println("Determine the optimal routes for 3-coalitions ...");
        id = startId;
        for (HashSet<Integer> key : Coalitions.keySet()) {
            kr = Coalitions.get(key);
            Route r = allRoutes.get(kr);
            r.id = id;
            Routes.add(r);
            id ++;
        }
        allRoutes.clear();
        Coalitions.clear();

        //end
        iters += (double) iter/(10^6);
        tt = (System.currentTimeMillis() - startTime)/1000.0/60;
        double cn = (double) coaln/10000;
        System.out.printf("Enumerating LIFO 3-routes stopped: coal %.2fe4, iter %d, time %.2f min, routes %d %n",
                cn, iter, tt, Routes.size());
        return Routes;
    }


    public ArrayList<Route> ENUM_FourRoutes(int startId) {
        int id = 0, coaln = 0, kr = 0;
        long iter = 0;
        double best, tt;
        HashSet<Integer> coalKey;
        int[] coalition;

        HashMap<HashSet<Integer>, Integer> Coalitions = new HashMap<>();
        ArrayList<Route> allRoutes = new ArrayList<>();
        ArrayList<Route> Routes = new ArrayList<>();
        Route route, best_route;

        long startTime = System.currentTimeMillis();
        out:for (int i = 1; i <= NUM; i++) {
            for (int j = 1; j <= NUM; j++) {
                if (i == j) {
                    continue;
                }
                for (int k = 1; k <= NUM; k++) {
                    if (k == j || k == i) {
                        continue;
                    }
                    for (int m = 1; m <= NUM; m++) {
                        if (m == k || m == j || m == i) {
                            continue;
                        }
                        // 装载约束
                        if (instance.AllCustomers.get(i).demand + instance.AllCustomers.get(j).demand +
                                instance.AllCustomers.get(k).demand + instance.AllCustomers.get(m).demand
                                > capacity) {
                            continue;
                        }

                        //the best route for coalition
                        coalition = new int[]{i, j, k, m};
                        coaln ++;
                        best = Double.MAX_VALUE;
                        best_route = new Route();

                        for (int pick1 : coalition) {
                            for (int pick2 : coalition) {
                                if (pick1 == pick2 || instance.Graph[pick1][pick2].distance > instance.radius) {
                                    continue;
                                }
                                for (int pick3 : coalition) {
                                    if (pick1 == pick3 || pick2 == pick3 || instance.Graph[pick2][pick3].distance >
                                            instance.radius) {
                                        continue;
                                    }
                                    for (int pick4 : coalition) {
                                        if (pick1 == pick4 || pick2 == pick4 || pick3 == pick4 || instance.Graph[pick3][pick4].distance
                                                > instance.radius) {
                                            continue;
                                        }
                                        for (int deli1 : coalition) {
                                            for (int deli2 : coalition) {
                                                if (deli1 == deli2 || instance.Graph[deli1 + NUM][deli2 + NUM].distance
                                                        > instance.radius) {
                                                    continue;
                                                }
                                                for (int deli3 : coalition) {
                                                    if (deli1 == deli3 || deli2 == deli3 || instance.Graph[deli2 + NUM][deli3 + NUM].distance
                                                            > instance.radius) {
                                                        continue;
                                                    }
                                                    for (int deli4 : coalition) {
                                                        if (deli1 == deli4 || deli2 == deli4 || deli3 == deli4
                                                                || instance.Graph[deli3 + NUM][deli4 + NUM].distance
                                                                > instance.radius) {
                                                            continue;
                                                        }

                                                        route = new Route();
                                                        route.R.add(0);
                                                        route.R.add(pick1);
                                                        route.R.add(pick2);
                                                        route.R.add(pick3);
                                                        route.R.add(pick4);
                                                        route.R.add(deli1 + NUM);
                                                        route.R.add(deli2 + NUM);
                                                        route.R.add(deli3 + NUM);
                                                        route.R.add(deli4 + NUM);
                                                        route.R.add(2 * NUM + 1);

                                                        //select the best route
                                                        if (instance.feasible(route,false) && route.cost < best) {
                                                            best = route.cost;
                                                            best_route = route.cloneRoute();
                                                        }
                                                        iter ++;
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        //Check economic validity
                        double LTLs = instance.SignleCost[i] + instance.SignleCost[j] + instance.SignleCost[k] +
                                instance.SignleCost[m];
                        if (best != Double.MAX_VALUE && best < LTLs) {
                            best_route.id = id;
                            allRoutes.add(best_route);

                            //store the valid coalition
                            coalKey = new HashSet<>();
                            coalKey.add(i); coalKey.add(j); coalKey.add(k); coalKey.add(m);
                            if (Coalitions.get(coalKey) == null) {
                                Coalitions.put(coalKey, id);
                            } else {
                                kr = Coalitions.get(coalKey); //route id
                                Route r = allRoutes.get(kr); //previous optimum route
                                if (best < r.cost) {
                                    Coalitions.replace(coalKey, id);
                                }
                            }
                            id ++;
                        }

                        tt = (System.currentTimeMillis() - startTime)/1000.0/60;
                        if (tt > maxTime) {
                            break out; //exceeding the time limit
                        }
                    }
                }
            }
        }

        // Select the optimal routes
        System.out.println("Determine the optimal routes for 4-coalitions ...");
        id = startId;
        for (HashSet<Integer> key : Coalitions.keySet()) {
            kr = Coalitions.get(key);
            Route r = allRoutes.get(kr);
            r.id = id;
            Routes.add(r);
            id ++;
        }
        allRoutes.clear();
        Coalitions.clear();

        //end
        iters += (double) iter/(10^6);
        tt = (System.currentTimeMillis() - startTime)/1000.0/60;
        double cn = (double) coaln/10000;
        System.out.printf("Enumerating 4-routes stopped: coal %.2fe4, iter %d, time %.2f min, routes %d %n",
                cn, iter, tt, Routes.size());
        return Routes;
    }


    public ArrayList<Route> ENUM_FourRoutes_LIFO(int startId) {
        int id = 0, coaln = 0, kr = 0;
        long iter = 0;
        double best, tt;
        HashSet<Integer> coalKey;
        int[] coalition;

        HashMap<HashSet<Integer>, Integer> Coalitions = new HashMap<>();
        ArrayList<Route> allRoutes = new ArrayList<>();
        ArrayList<Route> Routes = new ArrayList<>();
        Route route, best_route;

        long startTime = System.currentTimeMillis();
        out:for (int i = 1; i <= NUM; i++) {
            for (int j = 1; j <= NUM; j++) {
                if (i == j) {
                    continue;
                }
                for (int k = 1; k <= NUM; k++) {
                    if (k == j || k == i) {
                        continue;
                    }
                    for (int m = 1; m <= NUM; m++) {
                        if (m == k || m == j || m == i) {
                            continue;
                        }
                        // 装载约束
                        if (instance.AllCustomers.get(i).demand + instance.AllCustomers.get(j).demand +
                                instance.AllCustomers.get(k).demand + instance.AllCustomers.get(m).demand > capacity) {
                            continue;
                        }

                        //the best route for coalition
                        coalition = new int[]{i, j, k, m};
                        coaln ++;
                        best = Double.MAX_VALUE;
                        best_route = new Route();

                        for (int pick1 : coalition) {
                            for (int pick2 : coalition) {
                                if (pick1 == pick2 || instance.Graph[pick1][pick2].distance > instance.radius) {
                                    continue;
                                }
                                for (int pick3 : coalition) {
                                    if (pick1 == pick3 || pick2 == pick3 || instance.Graph[pick2][pick3].distance
                                            > instance.radius) {
                                        continue;
                                    }
                                    for (int pick4 : coalition) {
                                        if (pick1 == pick4 || pick2 == pick4 || pick3 == pick4 || instance.Graph[pick3][pick4].distance
                                                > instance.radius) {
                                            continue;
                                        }
                                        //LIFO
                                        if(instance.Graph[pick4+NUM][pick3+NUM].distance > instance.radius ||
                                                instance.Graph[pick3+NUM][pick2+NUM].distance > instance.radius ||
                                                instance.Graph[pick2+NUM][pick1+NUM].distance > instance.radius) {
                                            continue;
                                        }

                                        route = new Route();
                                        route.R.add(0);
                                        route.R.add(pick1);
                                        route.R.add(pick2);
                                        route.R.add(pick3);
                                        route.R.add(pick4);
                                        route.R.add(pick4 + NUM);
                                        route.R.add(pick3 + NUM);
                                        route.R.add(pick2 + NUM);
                                        route.R.add(pick1 + NUM);
                                        route.R.add(2 * NUM + 1);

                                        //select the best route
                                        if (instance.feasible(route,false) && route.cost < best) {
                                            best = route.cost;
                                            best_route = route.cloneRoute();
                                        }
                                        iter ++;
                                    }
                                }
                            }
                        }

                        //Check economic validity
                        double LTLs = instance.SignleCost[i] + instance.SignleCost[j] + instance.SignleCost[k] +
                                instance.SignleCost[m];
                        if (best != Double.MAX_VALUE && best < LTLs) {
                            best_route.id = id;
                            allRoutes.add(best_route);

                            //store the valid coalition
                            coalKey = new HashSet<>();
                            coalKey.add(i); coalKey.add(j); coalKey.add(k); coalKey.add(m);
                            if (Coalitions.get(coalKey) == null) {
                                Coalitions.put(coalKey, id);
                            } else {
                                kr = Coalitions.get(coalKey); //route id
                                Route r = allRoutes.get(kr); //previous optimum route
                                if (best < r.cost) {
                                    Coalitions.replace(coalKey, id);
                                }
                            }
                            id ++;
                        }

                        tt = (System.currentTimeMillis() - startTime)/1000.0/60;
                        if (tt > maxTime) {
                            break out; //exceeding the time limit
                        }
                    }
                }
            }
        }

        // Select the optimal routes
        System.out.println("Determine the optimal routes for 4-coalitions ...");
        id = startId;
        for (HashSet<Integer> key : Coalitions.keySet()) {
            kr = Coalitions.get(key);
            Route r = allRoutes.get(kr);
            r.id = id;
            Routes.add(r);
            id ++;
        }
        allRoutes.clear();
        Coalitions.clear();

        //end
        iters += (double) iter/(10^6);
        tt = (System.currentTimeMillis() - startTime)/1000.0/60;
        double cn = (double) coaln/10000;
        System.out.printf("Enumerating LIFO 4-routes stopped: coal %.2fe4, iter %d, time %.2f min, routes %d %n",
                cn, iter, tt, Routes.size());
        return Routes;
    }


    public ArrayList<Route> ENUM_FiveRoutes(int startId) {
        int id = 0, coaln = 0, kr = 0;
        long iter = 0;
        double best, tt;
        HashSet<Integer> coalKey;
        int[] coalition;

        HashMap<HashSet<Integer>, Integer> Coalitions = new HashMap<>();
        ArrayList<Route> allRoutes = new ArrayList<>();
        ArrayList<Route> Routes = new ArrayList<>();
        Route route, best_route;

        long startTime = System.currentTimeMillis();
        out:for (int i = 1; i <= NUM; i++) {
            for (int j = 1; j <= NUM; j++) {
                if (i == j) {
                    continue;
                }
                for (int k = 1; k <= NUM; k++) {
                    if (k == j || k == i) {
                        continue;
                    }
                    for (int m = 1; m <= NUM; m++) {
                        if (m == k || m == j || m == i) {
                            continue;
                        }
                        for (int n = 1; n <= NUM; n++) {
                            if (n == m || n == k || n == j || n == i) {
                                continue;
                            }
                            // 装载约束
                            if (instance.AllCustomers.get(i).demand + instance.AllCustomers.get(j).demand +
                                    instance.AllCustomers.get(k).demand + instance.AllCustomers.get(m).demand
                                    + instance.AllCustomers.get(n).demand > capacity) {
                                continue;
                            }

                            //the best route for coalition
                            coalition = new int[]{i, j, k, m, n};
                            coaln ++;
                            best = Double.MAX_VALUE;
                            best_route = new Route();

                            for (int pick1 : coalition) {
                                for (int pick2 : coalition) {
                                    if (pick1 == pick2 || instance.Graph[pick1][pick2].distance > instance.radius) {
                                        continue;
                                    }
                                    for (int pick3 : coalition) {
                                        if (pick1 == pick3 || pick2 == pick3 || instance.Graph[pick2][pick3].distance >
                                                instance.radius) {
                                            continue;
                                        }
                                        for (int pick4 : coalition) {
                                            if (pick1 == pick4 || pick2 == pick4 || pick3 == pick4 || instance.Graph[pick3][pick4].distance
                                                    > instance.radius) {
                                                continue;
                                            }
                                            for (int pick5 : coalition) {
                                                if (pick1 == pick5 || pick2 == pick5 || pick3 == pick5 || pick4 == pick5
                                                        || instance.Graph[pick4][pick5].distance > instance.radius) {
                                                    continue;
                                                }
                                                for (int deli1 : coalition) {
                                                    for (int deli2 : coalition) {
                                                        if (deli1 == deli2 || instance.Graph[deli1 + NUM][deli2 + NUM].distance
                                                                > instance.radius) {
                                                            continue;
                                                        }
                                                        for (int deli3 : coalition) {
                                                            if (deli1 == deli3 || deli2 == deli3 || instance.Graph[deli2 + NUM][deli3 + NUM].distance
                                                                    > instance.radius) {
                                                                continue;
                                                            }
                                                            for (int deli4 : coalition) {
                                                                if (deli1 == deli4 || deli2 == deli4 || deli3 == deli4
                                                                        || instance.Graph[deli3 + NUM][deli4 + NUM].distance
                                                                        > instance.radius) {
                                                                    continue;
                                                                }
                                                                for (int deli5 : coalition) {
                                                                    if (deli1 == deli5 || deli2 == deli5 || deli3 == deli5 || deli4 == deli5
                                                                            || instance.Graph[deli4 + NUM][deli5 + NUM].distance
                                                                            > instance.radius) {
                                                                        continue;
                                                                    }

                                                                    route = new Route();
                                                                    route.R.add(0);
                                                                    route.R.add(pick1);
                                                                    route.R.add(pick2);
                                                                    route.R.add(pick3);
                                                                    route.R.add(pick4);
                                                                    route.R.add(pick5);
                                                                    route.R.add(deli1 + NUM);
                                                                    route.R.add(deli2 + NUM);
                                                                    route.R.add(deli3 + NUM);
                                                                    route.R.add(deli4 + NUM);
                                                                    route.R.add(deli5 + NUM);
                                                                    route.R.add(2 * NUM + 1);

                                                                    //select the best route
                                                                    if (instance.feasible(route, false) && route.cost < best) {
                                                                        best = route.cost;
                                                                        best_route = route.cloneRoute();
                                                                    }
                                                                    iter++;
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }


                            //Check economic validity
                            double LTLs = instance.SignleCost[i] + instance.SignleCost[j] + instance.SignleCost[k] +
                                    instance.SignleCost[m] + instance.SignleCost[n];
                            if (best != Double.MAX_VALUE && best < LTLs) {
                                best_route.id = id;
                                allRoutes.add(best_route);

                                //store the valid coalition
                                coalKey = new HashSet<>();
                                coalKey.add(i); coalKey.add(j); coalKey.add(k);
                                coalKey.add(m); coalKey.add(n);
                                if (Coalitions.get(coalKey) == null) {
                                    Coalitions.put(coalKey, id);
                                } else {
                                    kr = Coalitions.get(coalKey); //route id
                                    Route r = allRoutes.get(kr); //previous optimum route
                                    if (best < r.cost) {
                                        Coalitions.replace(coalKey, id);
                                    }
                                }
                                id ++;
                            }

                            tt = (System.currentTimeMillis() - startTime)/1000.0/60;
                            if (tt > maxTime) {
                                break out; //exceeding the time limit
                            }
                        }
                    }
                }
            }
        }

        // Select the optimal routes
        System.out.println("Determine the optimal routes for 5-coalitions ...");
        id = startId;
        for (HashSet<Integer> key : Coalitions.keySet()) {
            kr = Coalitions.get(key);
            Route r = allRoutes.get(kr);
            r.id = id;
            Routes.add(r);
            id ++;
        }
        allRoutes.clear();
        Coalitions.clear();

        //end
        iters += (double) iter/(10^6);
        tt = (System.currentTimeMillis() - startTime)/1000.0/60;
        double cn = (double) coaln/10000;
        System.out.printf("Enumerating 5-routes stopped: coal %.2fe4, iter %d, time %.2f min, routes %d %n",
                cn, iter, tt, Routes.size());
        return Routes;
    }


    public ArrayList<Route> ENUM_FiveRoutes_LIFO(int startId) {
        int id = 0, coaln = 0, kr = 0;
        long iter = 0;
        double best, tt;
        HashSet<Integer> coalKey;
        int[] coalition;

        HashMap<HashSet<Integer>, Integer> Coalitions = new HashMap<>();
        ArrayList<Route> allRoutes = new ArrayList<>();
        ArrayList<Route> Routes = new ArrayList<>();
        Route route, best_route;

        long startTime = System.currentTimeMillis();
        out:for (int i = 1; i <= NUM; i++) {
            for (int j = 1; j <= NUM; j++) {
                if (i == j) {
                    continue;
                }
                for (int k = 1; k <= NUM; k++) {
                    if (k == j || k == i) {
                        continue;
                    }
                    for (int m = 1; m <= NUM; m++) {
                        if (m == k || m == j || m == i) {
                            continue;
                        }
                        for (int n = 1; n <= NUM; n++) {
                            if (n == m || n == k || n == j || n == i) {
                                continue;
                            }
                            // 装载约束
                            if (instance.AllCustomers.get(i).demand + instance.AllCustomers.get(j).demand +
                                    instance.AllCustomers.get(k).demand + instance.AllCustomers.get(m).demand
                                    + instance.AllCustomers.get(n).demand > capacity) {
                                continue;
                            }

                            //the best route for coalition
                            coalition = new int[]{i, j, k, m, n};
                            coaln ++;
                            best = Double.MAX_VALUE;
                            best_route = new Route();

                            for (int pick1 : coalition) {
                                for (int pick2 : coalition) {
                                    if (pick1 == pick2 || instance.Graph[pick1][pick2].distance > instance.radius) {
                                        continue;
                                    }
                                    for (int pick3 : coalition) {
                                        if (pick1 == pick3 || pick2 == pick3 || instance.Graph[pick2][pick3].distance
                                                > instance.radius) {
                                            continue;
                                        }
                                        for (int pick4 : coalition) {
                                            if (pick1 == pick4 || pick2 == pick4 || pick3 == pick4 || instance.Graph[pick3][pick4].distance
                                                    > instance.radius) {
                                                continue;
                                            }
                                            for (int pick5 : coalition) {
                                                if (pick1 == pick5 || pick2 == pick5 || pick3 == pick5 || pick4 == pick5
                                                        || instance.Graph[pick4][pick5].distance > instance.radius) {
                                                    continue;
                                                }
                                                //LIFO
                                                if (instance.Graph[pick5 + NUM][pick4 + NUM].distance > instance.radius ||
                                                        instance.Graph[pick4 + NUM][pick3 + NUM].distance > instance.radius ||
                                                        instance.Graph[pick3 + NUM][pick2 + NUM].distance > instance.radius ||
                                                        instance.Graph[pick2 + NUM][pick1 + NUM].distance > instance.radius) {
                                                    continue;
                                                }

                                                route = new Route();
                                                route.R.add(0);
                                                route.R.add(pick1);
                                                route.R.add(pick2);
                                                route.R.add(pick3);
                                                route.R.add(pick4);
                                                route.R.add(pick5);
                                                route.R.add(pick5 + NUM);
                                                route.R.add(pick4 + NUM);
                                                route.R.add(pick3 + NUM);
                                                route.R.add(pick2 + NUM);
                                                route.R.add(pick1 + NUM);
                                                route.R.add(2 * NUM + 1);

                                                //select the best route
                                                if (instance.feasible(route, false) && route.cost < best) {
                                                    best = route.cost;
                                                    best_route = route.cloneRoute();
                                                }
                                                iter++;
                                            }
                                        }
                                    }
                                }
                            }

                            //Check economic validity
                            double LTLs = instance.SignleCost[i] + instance.SignleCost[j] + instance.SignleCost[k] +
                                    instance.SignleCost[m] + instance.SignleCost[n];
                            if (best != Double.MAX_VALUE && best < LTLs) {
                                best_route.id = id;
                                allRoutes.add(best_route);

                                //store the valid coalition
                                coalKey = new HashSet<>();
                                coalKey.add(i); coalKey.add(j); coalKey.add(k);
                                coalKey.add(m); coalKey.add(n);
                                if (Coalitions.get(coalKey) == null) {
                                    Coalitions.put(coalKey, id);
                                } else {
                                    kr = Coalitions.get(coalKey); //route id
                                    Route r = allRoutes.get(kr); //previous optimum route
                                    if (best < r.cost) {
                                        Coalitions.replace(coalKey, id);
                                    }
                                }
                                id ++;
                            }

                            tt = (System.currentTimeMillis() - startTime)/1000.0/60;
                            if (tt > maxTime) {
                                break out; //exceeding the time limit
                            }
                        }
                    }
                }
            }
        }

        // Select the optimal routes
        System.out.println("Determine the optimal routes for 5-coalitions ...");
        id = startId;
        for (HashSet<Integer> key : Coalitions.keySet()) {
            kr = Coalitions.get(key);
            Route r = allRoutes.get(kr);
            r.id = id;
            Routes.add(r);
            id ++;
        }
        allRoutes.clear();
        Coalitions.clear();

        //end
        iters += (double) iter/(10^6);
        tt = (System.currentTimeMillis() - startTime)/1000.0/60;
        double cn = (double) coaln/10000;
        System.out.printf("Enumerating LIFO 5-routes stopped: coal %.2fe4, iter %d, time %.2f min, routes %d %n",
                cn, iter, tt, Routes.size());
        return Routes;
    }




}
