package PickandDelivery.PDP2_opt;

import Common.*;
import static AlgoRun.algoParam.*;

import java.text.ParseException;
import java.util.*;

public class PDP2opt {
    public Instance instance;
    public HashSet<Integer> alreadyServ; //ID of orders alreadyServ served
    public int NUM;
    public int num_MSTL;
    public int num_iter;
    public int num_opt;
    public double eps;


    public PDP2opt(Instance instance) {
        this.instance = instance;
        this.NUM = instance.NUM;
    }


    public void run() throws ParseException {
        long startTime = System.currentTimeMillis();
        eps = 1e-5;
        
        //greedy initialization stage
        alreadyServ = new HashSet<>();
        HashMap<Integer,Route> Routes = initialRoute();

        //2-opt improving stage;
        instance.routesOpt = optImprove(Routes);
        postProcess();

        //Algorithm stopped
        long endTime = System.currentTimeMillis();
        double tt = (endTime - startTime)/1000.0/60;
        System.out.printf("-------- The 2-opt heuristic is completed with %.4f min: %d routes, %d iter, " +
                "%d operations --------%n", tt, num_MSTL, num_iter, num_opt);

        //Record the results
        instance.Swap_Opt = num_opt;
        instance.ITER_Opt = num_iter;
        instance.TIME_Opt = tt;

        //clear temp data
        alreadyServ.clear();
    }


    //Postprocess routes
    public void postProcess () {
        for (Route route:instance.routesOpt.values()) {
            if (route.R.size() <= 4)
                continue;

            double LTL = 0;
            for (int i = 1; i < route.R.size()/2; i++) {
                int id = route.R.get(i);
                LTL += instance.SignleCost[id];
            }

            if (route.cost - LTL > eps) {
                //remove the nonoptimal routes
                instance.routesOpt.remove(route.id,route);
                System.out.printf("###### Opt route of size %d is NOT rational: %.4f, %.4f ###### %n",
                        route.R.size(), route.cost, LTL);

                //Add an LTL route
                for (int i = 1; i < route.R.size()/2; i++) {
                    int id = route.R.get(i);
                    Customer cus = instance.AllCustomers.get(id);

                    Route r = new Route();
                    r.R = new ArrayList<>(Arrays.asList(0,id,id+NUM,2*NUM+1));
                    r.id = instance.routesOpt.size();
                    r.cost = instance.SignleCost[i];
                    r.load = cus.demand;
                    instance.routesOpt.put(r.id, r);
                }
            }
        }
    }


    // The greedy initialization stage
    public HashMap<Integer,Route> initialRoute() throws ParseException {
        int LTL = 0, MSTL2 = 0, MSTL3M = 0, rid = 0;
        HashMap<Integer, Route> Routes = new HashMap<>();

        //*** AllCustomers.size()=2*NUM+2, AllCustomers[0] = 0, AllCustomers[2*NUM+1] = 2*NUM+1
        for (int i = 1; i <= NUM - 1; i++) { //*** double check???? ***
            Customer first = instance.AllCustomers.get(i);
            if (alreadyServ.contains(i)) {
                continue; //order id is included at the insertion phase
            }
            boolean assign = false;

            for (int j = 1; j <= NUM; j++) {//*** double check???? ***
                if (assign) {
                    break; //move to the next index i
                }

                Customer second = instance.AllCustomers.get(j);
                if (alreadyServ.contains(j) || j == i) {
                    continue; //select the next j
                }

                //check capacity limit
                if (first.demand + second.demand > capacity) {
                    continue; //select the next j
                }

                //Check bundling radius
                if (instance.Graph[i][j].distance > instance.radius && instance.Graph[j][i].distance
                        > instance.radius) {
                    continue; //select the next j
                }
                if (instance.Graph[i+NUM][j+NUM].distance > instance.radius
                        && instance.Graph[j+NUM][i+NUM].distance > instance.radius) {
                    continue; //select the next j
                }

                //Initial route (i,j)
                Route route = constructTwo(first, second);

                //Greedy expanding route
                if (route.R.size() > 4) {
                    route.id = rid;
                    Route routeInsert = greedyInsert(route);
                    Routes.put(rid, routeInsert);

                    for (int k = 1; k < routeInsert.R.size()/2; k++) {
                        int id = routeInsert.R.get(k);
                        Customer cus = instance.AllCustomers.get(id);
                        cus.route = rid;
                        instance.AllCustomers.get(cus.id+NUM).route = rid;
                        alreadyServ.add(cus.id); //the order is served
                    }

                    if (routeInsert.R.size() <= 6) {
                        MSTL2 ++;
                    } else {
                        MSTL3M ++;
                    }

                    assign = true; //the current order i is bundled with some j
                    rid ++;
                }
            }
        }

        
        //Assign remained orders to LTL routes
        for (int i = 1; i <= NUM; i++) { //*** double check???? ***
            if (!alreadyServ.contains(i)) {
                Customer cus = instance.AllCustomers.get(i);
                Route route = new Route();
                route.R = new ArrayList<>(Arrays.asList(0,i,i+NUM,2*NUM+1));
                //set route information
                route.cost = instance.SignleCost[i];
                route.load = cus.demand;

                //update the sets
                route.id = rid;
                Routes.put(rid,route);
                cus.route = rid;
                instance.AllCustomers.get(i+NUM).route = rid;
                LTL ++;
                rid ++;
            }
        }

        //The set of initialized routes
        num_MSTL = MSTL2 + MSTL3M;
        instance.OPTNum = LTL + num_MSTL;
        System.out.printf("The 2-opt heuristic initialized with: %d LTL routes, %d MSTL2 routes, %d MSTL3+ routes %n",
                LTL, MSTL2, MSTL3M);
        return Routes;
    }


    //The 2-opt improving stages
    public HashMap<Integer,Route> optImprove(HashMap<Integer,Route> Routes){
        //Initial costs
        double Cost0 = 0;
        for(Route route:Routes.values()){
            Cost0 = Cost0 + route.cost;
        }
        num_iter = 0;
        num_opt = 0;

        //2-opt improvement
        HashMap<Integer, Route> routeSol = new HashMap<>();
        for (Route route:Routes.values()) {
            int rid = route.id;
            Route route_opt = route.cloneRoute();

            if (route.R.size() > 4) { //only improve MSTL routes
                //Improve the pickup sequence
                route_opt = swapOpt(route,"pickup");

                //Improve the delivery sequence
                if (!instance.LIFO) {
                    route = route_opt.cloneRoute();
                    route_opt = swapOpt(route,"delivery");
                }
            }
            routeSol.put(rid,route_opt);

            //Update the service information of orders
            for (int k = 1; k < route_opt.R.size()/2; k++) {
                Customer cus = instance.AllCustomers.get(route_opt.R.get(k));
                cus.route = rid;
                instance.AllCustomers.get(cus.id+NUM).route = rid;
            }
        }

        //The improvement
        double Cost = 0;
        for (Route route:routeSol.values()) {
            Cost = Cost + route.cost;
        }
        double impr_org = 100*(instance.OriginCost - Cost)/instance.OriginCost;
        double impr_opt = 100*(Cost0 - Cost)/Cost0;
        System.out.printf("The 2-opt routes are derived with: LTL %.4f%%, initial %.4f%%, %d iterations, %d operations %n",
                impr_org, impr_opt, num_iter, num_opt);

        //Output the new routes
        return routeSol;
    }


    //The initial two-order MSTL routes
    public Route constructTwo(Customer first,Customer second){
        int i = first.id, j = second.id;
        double LTLs = instance.SignleCost[i] + instance.SignleCost[j]; //Check economic validity

        Route route = new Route();
        route.cost = Double.MAX_VALUE;

        //Case: LIFO policy
        if (instance.LIFO) {
            Route r1 = new Route();
            r1.R = new ArrayList<>(Arrays.asList(0,i,j,j+NUM,i+NUM,2*NUM+1));
            if (instance.bundle_feasible(r1) && instance.feasible(r1,false) && r1.cost < LTLs) {
                route = r1.cloneRoute();
                return route;
            }

            Route r2 = new Route();
            r2.R = new ArrayList<>(Arrays.asList(0,j,i,i+NUM,j+NUM,2*NUM+1));
            if (instance.bundle_feasible(r2) && instance.feasible(r2,false) && r2.cost < LTLs) {
                route = r2.cloneRoute();
                return route;
            }
            //Case: Flexible policy
        } else {
            Route r1 = new Route();
            r1.R = new ArrayList<>(Arrays.asList(0,i,j,j+NUM,i+NUM,2*NUM+1));
            if (instance.bundle_feasible(r1) && instance.feasible(r1,false) && r1.cost < LTLs) {
                route = r1.cloneRoute();
                return route;
            }

            Route r2 = new Route();
            r2.R = new ArrayList<>(Arrays.asList(0,i,j,i+NUM,j+NUM,2*NUM+1));
            if (instance.bundle_feasible(r2) && instance.feasible(r2,false) && r2.cost < LTLs) {
                route = r2.cloneRoute();
                return route;
            }

            Route r3 = new Route();
            r3.R = new ArrayList<>(Arrays.asList(0,j,i,i+NUM,j+NUM,2*NUM+1));
            if (instance.bundle_feasible(r3) && instance.feasible(r3,false) && r3.cost < LTLs) {
                route = r3.cloneRoute();
                return route;
            }

            Route r4 = new Route();
            r4.R = new ArrayList<>(Arrays.asList(0,j,i,j+NUM,i+NUM,2*NUM+1));
            if (instance.bundle_feasible(r4) && instance.feasible(r4,false) && r4.cost < LTLs) {
                route = r4.cloneRoute();
                return route;
            }
        }

        //Double check rationality
        double LTL = 0;
        for (int k = 1; k < route.R.size()/2; k++) {
            int id = route.R.get(k);
            LTL += instance.SignleCost[id];
        }
        if (instance.feasible(route,false) && route.cost - LTL >= 1e-5) {
            route = new Route();
        }
        // Select the best route for the two orders
        return route;
    }


    //Expand the two-order route by greedy insertion
    public Route greedyInsert(Route route){
        //Initialize the route
        Route route_insert = route.cloneRoute();

        //The ID of orders not served
        ArrayList<Integer> unServ = new ArrayList<>();
        for (int i = 1; i <= NUM; i++) {
            if (!alreadyServ.contains(i)) {
                unServ.add(i);
            }
        }

        //Initial LTL cost
        double LTLs = 0, load = 0;
        for (int k = 1; k < route_insert.R.size()/2; k++) {
            int id = route_insert.R.get(k);
            LTLs += instance.SignleCost[id];
            load += instance.AllCustomers.get(id).demand;
        }

        //Greedy insertion
        int onboard = 2, k = 0;
        while (onboard < instance.maxOrders && k < unServ.size()) {
            int id = unServ.get(k); //order ID
            Customer cus = instance.AllCustomers.get(id);

            if (load + cus.demand > capacity) {
                k ++;
                continue;
            }

            //Add a new order into the best positions
            // e.g., R=(0,i,j,n+j,n+i,2n+1) -----> R'=(0,i,j,k,n+k,n+j,n+i,2n+1)
            // route.R.size() = 6, route.R'.size() = 8
            // Add an order into R: feasible pick positions = route.R.size() / 2
            // By LIFO, pick + deli = route.R.size()+1
            // By flexible, 1<=pick<=route.R.size()/2; route.R.size()/2+1<=deli<=route.R.size()

            //record the best insertion
            Route route_best = new Route();
            route_best.cost = Double.MAX_VALUE;
            if (instance.LIFO) { //Case: LIFO
                for (int p = 1; p <= route_insert.R.size()/2; p++) {
                    int d = route_insert.R.size() + 1 - p;
                    Route route_clone = route_insert.cloneRoute();
                    route_clone.R.add(p, cus.id);
                    route_clone.R.add(d,cus.id+NUM);
                    if (instance.bundle_feasible(route_clone) && instance.feasible(route_clone,false)
                            && route_clone.cost < route_best.cost) {
                        route_best = route_clone.cloneRoute();
                    }
                }

            } else { //Case: Flexible
                for (int p = 1; p <= route_insert.R.size()/2; p++) {
                    for (int d = route_insert.R.size()/2 + 1; d <= route_insert.R.size(); d++) {
                        Route route_clone = route_insert.cloneRoute();
                        route_clone.R.add(p, cus.id);
                        route_clone.R.add(d,cus.id+NUM);
                        if (instance.bundle_feasible(route_clone) && instance.feasible(route_clone,false)
                                && route_clone.cost < route_best.cost) {
                            route_best = route_clone.cloneRoute();
                        }
                    }
                }
            }

            //Select the best expansion
            if (route_best.R.size() > route_insert.R.size() && route_best.cost < LTLs + instance.SignleCost[id]) {
                route_insert = route_best.cloneRoute();
                onboard = onboard + 1;

                //if feasible, remove the inserted order and restart
                unServ.remove(k);
                k = 0;
                LTLs = LTLs + instance.SignleCost[id];
                load = load + cus.demand;
            } else {
                //not feasible search the next order
                k ++;
            }

        }  //end

        //double check rationality
        LTLs = 0;
        for (int i = 1; i < route_insert.R.size()/2; i++) {
            int id = route_insert.R.get(i);
            LTLs += instance.SignleCost[id];
        }
        if (instance.feasible(route_insert,false) && route_insert.cost >= LTLs) {
            route_insert = route.cloneRoute();
        }

        /*if (route_insert.R.size() > route.R.size()) {
            System.out.printf("Insert %d orders successfully into route ID %d %n", num, route.id);
        }*/
        return route_insert;
    }


    //First-improvement search strategy
    public Route swapOpt(Route route,String task){ //*** double check???? ***
        Route route_opt = route.cloneRoute();
        ArrayList<Integer> path, picks, delis;
        int swaps = 1, iter = 1, num = 0;
        int E = 2*NUM + 1;

        //2-opt on the pickup path subroute
        if (task.equals("pickup")) {
            while (swaps > 0) { //loop until no improvements are made.
                swaps = 0;
                //the pickup path subroute: 0 -> i1 -> i2 -> 2*N+1
                path = new ArrayList<>();
                for (int k = 0; k < route_opt.R.size()/2; k++) {
                    path.add(route_opt.R.get(k));
                }
                path.add(E);

                //the delivery subroute: N+i1 -> N+i2 -> 2*N+1
                delis = new ArrayList<>(); //include depot 2*NUM + 1
                for (int k = route_opt.R.size()/2; k < route_opt.R.size(); k++) {
                    delis.add(route_opt.R.get(k));
                }

                //perform 2-opt move
                for (int i = 0; i < path.size() - 3; i++) {
                    for (int j = i + 2; j < path.size() - 1; j++) {
                        Route route_new = new Route();
                        route_new.cost = Double.MAX_VALUE;

                        picks = swap(path,i,j);
                        if (instance.LIFO) { //update delivery sequence under LIFO
                            ArrayList<Integer> deli_lifo = new ArrayList<>();
                            for (int k = picks.size() - 2; k >= 1; k--) {
                                deli_lifo.add(picks.get(k) + NUM);
                            }
                            deli_lifo.add(2*NUM+1);
                            delis = deli_lifo;
                        }

                        //the new route
                        route_new.R.addAll(picks.subList(0,picks.size()-1));
                        route_new.R.addAll(delis);

                        //update and check feasibility
                        if (instance.bundle_feasible(route_new) && instance.feasible(route_new,false)
                                && route_new.cost < route_opt.cost) {
                            route_opt = route_new.cloneRoute();
                            swaps ++;
                            num ++;
                            break;
                        }
                    }
                } //end the search for the current route
                iter ++;
            }
        }

        //2-opt on the delivery path subroute
        if (task.equals("delivery")) {
            while (swaps > 0) { //loop until no improvements are made.
                swaps = 0;

                //the pickup path subroute: 0 -> i1 -> i2
                picks = new ArrayList<>(); //route.R.subList(0,route.R.size()/2);
                for (int k = 0; k < route_opt.R.size()/2; k++) {
                    picks.add(route_opt.R.get(k));
                }

                //the delivery subroute: 0 -> N+i1 -> N+i2 -> 2*N+1
                path = new ArrayList<>(); // include 2*N+1
                path.add(0);
                for (int k = route_opt.R.size()/2; k < route_opt.R.size(); k++) {
                    path.add(route_opt.R.get(k));
                }

                //perform 2-opt move
                for (int i = 0; i < path.size() - 3; i++) {
                    for (int j = i + 2; j < path.size()-1; j++) {
                        Route route_new = new Route();
                        route_new.cost = Double.MAX_VALUE;

                        //the new route
                        delis = swap(path,i,j);
                        route_new.R.addAll(picks);
                        route_new.R.addAll(delis.subList(1,delis.size()));

                        //update and check feasibility
                        if (instance.bundle_feasible(route_new) && instance.feasible(route_new,false)
                                && route_new.cost < route_opt.cost) {
                            route_opt = route_new.cloneRoute();
                            swaps ++;
                            num ++;
                            break;
                        }
                    }
                }
                //end the search for the current route
                iter ++;
            }
        }
        //end
        // System.out.printf("The 2-opt search in %s completed with: %d iterations and %d improving operations %n", task, iter, num);
        num_iter += iter;
        num_opt += num;
        return route_opt;
    }


    // 2-opt operator
    public ArrayList<Integer> swap(ArrayList<Integer> path,int i,int j){
        //Add path from L[0] to L[i]
        ArrayList<Integer> pre = new ArrayList<>(); //path.subList(0,i+1);
        for (int k = 0; k <= i; k++) {
            pre.add(path.get(k));
        }

        //Add the reversed sequence of path from L[i+1] to L[j]
        ArrayList<Integer> reserve = new ArrayList<>(); //path.subList(i+1,j+1);
        for (int k = i+1; k <= j; k++) {
            reserve.add(path.get(k));
        }
        Collections.reverse(reserve);

        //Add path from L[j+1] to last
        ArrayList<Integer> after = new ArrayList<>(); //path.subList(j+1,path.size());
        for (int k = j+1; k < path.size(); k++) {
            after.add(path.get(k));
        }

        //The new path
        ArrayList<Integer> opt = new ArrayList<>();
        opt.addAll(pre);
        opt.addAll(reserve);
        opt.addAll(after);

        //End
        return opt;
    }


}
