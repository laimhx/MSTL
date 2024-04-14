package PickandDelivery.PDPLabel;

import Common.*;
import static AlgoRun.algoParam.*;

import java.text.ParseException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;

//import gurobi.*;
import com.gurobi.gurobi.*;

public class PDP_label {
    public Instance instance;
    public HashSet<Label> alreadyLabels; //record of historical labels
    public ArrayList<Route> LTLRoutes;
    public HashSet<Label> negRoutes; //record of negative-cost labels
    public int NUM;
    public int numLab;
    public double eps;


    public PDP_label(Instance instance) {
        this.instance = instance;
        this.NUM = instance.NUM;
    }
    
    
    public void run() {
        long startTime = System.currentTimeMillis();
        eps = 1e-5;

        //the initial sets
        instance.CGRoutes = new ArrayList<>();

        //the CG algorithm
        CGprocedure();

        //the final results
        resultAnalysis(instance.CGRoutes);

        //Algorithm stopped
        long endTime = System.currentTimeMillis();
        double tt = (endTime - startTime)/1000.0/60;
        System.out.printf("--------Algorithm CG is completed with %.4f min, %d iterations, %d labels, Routes %d--------%n",
                tt, instance.ITER_label, numLab, instance.RCGNum);

        //Record the results
        instance.RCGNum = instance.CGRoutes.size();
        instance.TIME_label = tt;

        //clear temp data
    }
    

    public void CGprocedure() {
        //empty label sets
        alreadyLabels = new HashSet<>();
        negRoutes = new HashSet<>();

        //initialization by LTL routes
        LTLRoutes = initialRoutes();

        //Set initial bundling limit
        int maxB = Math.min(2, instance.maxOrders);
        System.out.printf("Set the initial bundling limit as B = %d with maximum as %d %n",
                maxB, instance.maxOrders);

        //start searching
        double[] pi = new double[NUM+1];
        double time_LP = 0, time_SP = 0, time_IP = 0;
        long tt0;
        int num_new = 1, Iter = 0, numlab2 = 0;
        boolean first = true, disp;
        while(num_new > 0 && negRoutes.size() <= MAXR) {
            disp = (NUM >= 1000 & Iter % 20 == 0);
            if (alreadyLabels.size() >= MAXLAB) {
                alreadyLabels.removeIf(label -> label.getSize() <= 2);
            }

            //dual pricing
            tt0 = System.currentTimeMillis();
            pi = SolveRMP(LTLRoutes,negRoutes);
            time_LP += (System.currentTimeMillis() - tt0)/1000.0/60;

            //start labeling procedure
            tt0 = System.currentTimeMillis();
            SP_label sp = new SP_label(instance,first,disp,maxB,pi,alreadyLabels,negRoutes);
            num_new = sp.run();
            time_SP += (System.currentTimeMillis() - tt0)/1000.0/60;

            //increase the size of bundling limit
            //boolean flag = (num_new == 0 || negRoutes.size() >= 75000);
            if (num_new == 0 && maxB < instance.maxOrders) {
                if (maxB == 2) {
                    numlab2 = alreadyLabels.size();
                }

                maxB ++;
                num_new = 1;
                for (Label lab : alreadyLabels) {
                    lab.reachNum += 1;
                }
                Iter ++;

                System.out.printf("********* Increase bundling limit to B = %d at iteration %d *********%n",
                        maxB, Iter);
                continue;
            }

            if (disp) {
                System.out.printf("CG at iteration %d: B = %d; newRoutes %d, labels %d, negRoutes %d %n",
                        Iter, maxB, num_new, alreadyLabels.size(), negRoutes.size());
            }

            first = false; //later round
            numLab = Math.max(numLab, alreadyLabels.size());
            Iter ++;
        }

        //Process the generated routes
        numLab = numlab2 + alreadyLabels.size();
        labRoutes();

        //clear temp data
        alreadyLabels.clear();
        negRoutes.clear();
        LTLRoutes.clear();

        //Solve the final integer master problem
        System.out.printf("Solving the final master problem with orders %d, routes %d, labels %d %n",
                NUM, instance.CGRoutes.size(), numLab);
        tt0 = System.currentTimeMillis();
        instance.routesLabel = SolveMP(instance.CGRoutes);
        time_IP = (System.currentTimeMillis() - tt0)/1000.0/60;

        //compute the relaxation gaps
        double costLP = SolveLP(instance.CGRoutes);
        double cost = 0;
        for(Route route : instance.routesLabel.values()){
            cost += route.cost;
        }

        //record the results
        instance.ITER_label = Iter;
        instance.gap_LP_label = 100*(cost - costLP)/costLP;
        instance.TIME_LP_label = time_LP;
        instance.TIME_SP_label = time_SP;
        instance.TIME_IP_label = time_IP;
        instance.CG_label = (double) numLab/(10^6);

        System.out.printf("At the final step of CG: relax %.4f%%, time_LP %.2f min, time_SP %.2f min, time_IP %.2f min %n",
                instance.gap_LP_label, time_LP, time_SP, time_IP);
    }


    // size of generated routes
    public void resultAnalysis(ArrayList<Route> Routes) {
        instance.RCG_2 = 0;
        instance.RCG_3 = 0;
        instance.RCG_4 = 0;
        instance.RCG_5M = 0;
        for(Route r : Routes){
            if(r.R.size() == 6){
                instance.RCG_2 ++;
            } else if (r.R.size() == 8) {
                instance.RCG_3 ++;
            } else if (r.R.size() == 10) {
                instance.RCG_4 ++;
            } else if (r.R.size() > 10) {
                instance.RCG_5M ++;
            }
        }

        System.out.printf("Total MSTL routes generated by CG: 2-,3-,4-,5M: %d, %d, %d, %d %n",
                instance.RCG_2, instance.RCG_3, instance.RCG_4, instance.RCG_5M);
    }
    
    
    public ArrayList<Route> initialRoutes() {
        ArrayList<Route> Routes = new ArrayList<>();
        int id = 0;

        //Add all the LTL routes
        for (int i = 1; i <= NUM; i++) {
            Route route = new Route();
            route.R.add(0);
            route.R.add(i);
            route.R.add(i + NUM);
            route.R.add(2 * NUM + 1);

            route.id = id;
            route.cost = instance.SignleCost[i];
            Routes.add(route);
            id ++;
        }

        System.out.printf("Initialize algorithm CG with %d LTL routes. %n", Routes.size());
        return Routes;
    }


    public void labRoutes() {
        //Add LTL routes
        instance.CGRoutes.addAll(LTLRoutes);

        //Add MSTL routes
        int id = LTLRoutes.size();
        for (Label lab : negRoutes) {
            Route route = new Route();
            route.R = new ArrayList<>();
            route.R.add(0);
            route.R.addAll(lab.Picks);

            ArrayList<Integer> Delis = setDelivery(lab);
            route.R.addAll(Delis);
            route.R.add(2 * NUM + 1);

            route.load = lab.load;

            //compute the cost of route
            double dist = 0;
            for (int i = 1; i < route.R.size() - 2; i++) {
                dist += instance.Graph[route.R.get(i)][route.R.get(i + 1)].distance;
            }
            route.cost = instance.Graph[route.R.get(1)][route.R.get(route.R.size()-2)].distance *
                    instance.Rate[route.R.get(1)][route.R.get(route.R.size() - 2)] + (route.R.size() - 4) *
                    instance.stop + (dist - instance.Graph[route.R.get(1)][route.R.get(route.R.size()-2)].distance) *
                    instance.detour;

            double LTLs = 0;
            for(int i : lab.Picks){
                LTLs += instance.SignleCost[i];
            }
            if (route.cost < LTLs) { //instance.checkRoute(route) &&
                route.id = id;
                instance.CGRoutes.add(route);
                id ++;
            }
        }
    }


    //Solve RMP by Gurobi
    public double[] SolveRMP(ArrayList<Route> LTL, HashSet<Label> MSTL) {
        double[] pi = new double[NUM+1];
        try {
            //Setup model
            GRBEnv env = new GRBEnv();
            GRBModel model = new GRBModel(env);
            GRBLinExpr expr;
            model.set(GRB.StringAttr.ModelName, "RMP");

            //Process the input data
            int nvar = LTL.size() + MSTL.size(), j = 0;
            double[] coeff = new double[nvar]; //coefficients
            double[][] Constr = new double[NUM + 1][nvar]; //Constraint matrix a_{ir}: row 0 are all zeros

            for (j = 0; j < LTL.size(); j++) { //LTL routes
                Route r = LTL.get(j);
                coeff[j] = r.cost;
                int i = r.R.get(1); //order id
                Constr[i][j] = 1;
            }

            j = LTL.size();
            for (Label lab : MSTL) { //MSTL routes
                coeff[j] = lab.routeCost;
                for(int i : lab.Picks){
                    Constr[i][j] = 1;
                }
                j ++;
            }

            //Decision variables: x_{r}
            GRBVar[] x = new GRBVar[nvar];
            for (j = 0; j < nvar; j++) {
                x[j] = model.addVar(0.0, 1.0, 0.0, GRB.CONTINUOUS, "x" + "_" + j);
            }
    
            //Objective function
            expr = new GRBLinExpr();
            for (j = 0; j < nvar; j++) { //LTL routes
                expr.addTerm(coeff[j], x[j]);
            }
            model.setObjective(expr, GRB.MINIMIZE);

            //Set-covering constraint: sum_{r}a_{ir}x_{r} >= 1, i = 1,...,N;
            //size: N x nr (row 0 of Constr is not used!)
            for (int i = 1; i <= NUM; i++) {
                expr = new GRBLinExpr();
                expr.addTerms(Constr[i], x);
                model.addConstr(expr, GRB.GREATER_EQUAL, 1.0, "cons" + i);
            }
    
            model.set(GRB.IntParam.OutputFlag, 0); //no log output
            model.set("Method", "2"); //1 = dual simplex; 2 = barrier method
            model.set("OptimalityTol", "1e-3"); //model.set("OptimalityTol", "1e-3")
            model.optimize();
    
            //Check validity of solution
            int status = model.get (GRB.IntAttr.Status);
            if (status == GRB.Status.INF_OR_UNBD || status == GRB.Status.INFEASIBLE){
                System.out.printf("###### The RMP pricing problem was infeasible: status %d ###### %n", status);
                System.exit(0);
            }
    
            //Dual solution
            double[] dual = model.get(GRB.DoubleAttr.Pi, model.getConstrs());
            for(j = 1; j <= NUM; j++) {
                pi[j] = dual[j-1]; //dual variables: Pi_0, Pi_1, ..., Pi_{n-1}
                if (Math.abs(pi[j]) <= eps) {
                    pi[j] = 0;
                }
            }
    
            //Clear the model
            model.dispose();
            env.dispose();
    
        } catch (GRBException e) {
            System.err.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
        }
        //end
        return pi;
    }

    
    //Solve MP by Gurobi
    public HashMap<Integer, Route> SolveMP(ArrayList<Route> ROUTES) {
        HashMap<Integer, Route> routeSol = new HashMap<>();
        try {
            //Setup model
            GRBEnv env = new GRBEnv();
            GRBModel model = new GRBModel(env);
            GRBLinExpr expr;
            model.set(GRB.StringAttr.ModelName, "MP");

            //Process data: constraint matrix a_{ir}
            double[][] Constr = new double[NUM + 1][ROUTES.size()]; //(NUM+1) x R
            for (int j = 0; j < ROUTES.size(); j++) {
                Route route = ROUTES.get(j);
                for (int k = 1; k < route.R.size() / 2; k++) {
                    int i = route.R.get(k);
                    Constr[i][j] = 1;
                }
            }

            //Decision variables
            GRBVar[] x = new GRBVar[ROUTES.size()];
            for (int j = 0; j < ROUTES.size(); j++) {
                x[j] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "x" + "_" + j);
            }
    
            //Objective function
            expr = new GRBLinExpr();
            for (int j = 0; j < ROUTES.size(); j++) {
                Route r = ROUTES.get(j);
                expr.addTerm(r.cost, x[j]);
            }
            model.setObjective(expr, GRB.MINIMIZE);

            //Set-covering constraint: sum_{r}a_{ir}x_{r} >= 1, i = 1,...,N;
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
                System.out.printf("###### The CG master problem was stopped with status %d ###### %n", status);
                System.exit(0);
            }
    
            //The solution
            for (int j = 0; j < ROUTES.size(); j++) {
                if (Math.abs(x[j].get(GRB.DoubleAttr.X) - 1) <= eps) {
                    Route r = ROUTES.get(j);
                    routeSol.put(r.id, r);
                }
            }

            //Clear the model
            model.dispose();
            env.dispose();
    
        } catch (GRBException e) {
            System.err.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
        }

        return routeSol;
    }


    public double SolveLP(ArrayList<Route> ROUTES) {
        double optCost = 0;
        try {
            //Setup model
            GRBEnv env = new GRBEnv();
            GRBModel model = new GRBModel(env);
            GRBLinExpr expr;
            model.set(GRB.StringAttr.ModelName, "MP");

            //Process data: constraint matrix a_{ir}
            double[][] Constr = new double[NUM + 1][ROUTES.size()]; //(NUM+1) x R
            for (int j = 0; j < ROUTES.size(); j++) {
                Route route = ROUTES.get(j);
                for (int k = 1; k < route.R.size() / 2; k++) {
                    int i = route.R.get(k);
                    Constr[i][j] = 1;
                }
            }

            //Decision variables
            GRBVar[] x = new GRBVar[ROUTES.size()];
            for (int j = 0; j < ROUTES.size(); j++) {
                x[j] = model.addVar(0.0, 1.0, 0.0, GRB.CONTINUOUS, "x" + "_" + j);
            }

            //Objective function
            expr = new GRBLinExpr();
            for (int j = 0; j < ROUTES.size(); j++) {
                Route r = ROUTES.get(j);
                expr.addTerm(r.cost, x[j]);
            }
            model.setObjective(expr, GRB.MINIMIZE);

            //Set-covering constraint: sum_{r}a_{ir}x_{r} >= 1, i = 1,...,N;
            for (int i = 1; i <= NUM; i++) {
                expr = new GRBLinExpr();
                expr.addTerms(Constr[i], x);
                // model.addConstr(expr, GRB.GREATER_EQUAL, 1.0, "cons" + i);
                model.addConstr(expr, GRB.EQUAL, 1.0, "cons" + i);
            }

            //The optimization parameters
            model.set(GRB.IntParam.OutputFlag, 0); //no log output
            model.set("Method", "2"); //1 = dual simplex; 2 = barrier method
            model.set("OptimalityTol", "1e-3"); //model.set("OptimalityTol", "1e-3")

            //Solve the model
            model.optimize();

            //Check validity of solution
            int status = model.get (GRB.IntAttr.Status);
            if (status == GRB.Status.INF_OR_UNBD || status == GRB.Status.INFEASIBLE){
                System.out.printf("###### The CG master problem was stopped with status %d ###### %n", status);
                System.exit(0);
            }

            //The solution
            optCost = model.get(GRB.DoubleAttr.ObjVal);

            //Clear the model
            model.dispose();
            env.dispose();

        } catch (GRBException e) {
            System.err.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
        }

        return optCost;
    }


    //Search the optimal delivery sequence
    public ArrayList<Integer> setDelivery(Label label){
        ArrayList<Integer> Delis = new ArrayList<>();
        int ordNum = label.Picks.size();

        int first = label.firstPickUpNode;
        Customer cus0 = instance.AllCustomers.get(first);
        double startTime = cus0.early - instance.Graph[0][first].duration;  //vehicle departure time from depot

        // Enumerate deliveries
        HashSet<ArrayList<Integer>> delis = new HashSet<>();
        if (instance.LIFO) {
            // Case: delivery sequence is reversed to pickup sequence
            ArrayList<Integer> deli = new ArrayList<>();
            for(int i = ordNum-1; i >= 0; i--)
                deli.add(label.Picks.get(i) + NUM);
            delis.add(deli);
        } else {
            // Case: enumerate the optimal delivery sequence
            delis = dfs(label.Picks,new ArrayList<>(),new HashSet<>(),new boolean[label.Picks.size()]);
        }

        // Check deliver feasibility
        label.linedetour = Double.MAX_VALUE;
        for (ArrayList<Integer> deli : delis) {
            double serveTime = label.StartServeTime; //at the current pick node
            double dist = 0; //initial by pickup subroute distance
            int pre = label.curNode; //the last delivery node,对于第一个deli点来说，是当前扩展pick点

            for (int c : deli) {
                // bundling radius limit
                if (pre > NUM && instance.Graph[pre][c].distance > instance.radius) {
                    serveTime = Double.MAX_VALUE;
                    break;
                }
                Customer cus = instance.AllCustomers.get(c);
                serveTime = serveTime + instance.AllCustomers.get(pre).serve / 60 +
                        instance.Graph[pre][c].duration;
                dist += instance.Graph[pre][c].distance;

                //delivery time window
                if(serveTime > cus.last) {
                    serveTime = Double.MAX_VALUE;
                    break;
                }
                pre = c;
            }

            //the last delivery node
            if (serveTime == Double.MAX_VALUE) { // infeasible label
                continue;
            }

            //hours of service limit
            int last = deli.getLast();
            double time = (serveTime + instance.AllCustomers.get(last).serve / 60 +
                    instance.Graph[last][2 * NUM + 1].duration) - startTime;
            if (time <= timelimit) {
                //the LH cost for end deli id
                double lh = instance.Graph[first][last].distance * (instance.Rate[first][last] - instance.detour);
                //stop-off and dual price are fixed by the picks
                double costs = lh + instance.detour * dist;

                if (costs < label.linedetour) {
                    label.linedetour = costs;
                    //Reset the nodes on the label routes
                    Delis.clear();
                    Delis.addAll(deli);
                }
            }
        }

        return Delis;
    }


    //Enumerate the sequence of deliveries
    public HashSet<ArrayList<Integer>> dfs(ArrayList<Integer> picks, ArrayList<Integer> deli,
                                           HashSet<ArrayList<Integer>> delis, boolean[] used){
        if (deli.size() == picks.size()) {
            delis.add(new ArrayList<>(deli));
            return delis;
        }

        for (int i = 0; i < picks.size(); i++) {
            if (used[i]) continue;
            deli.add(picks.get(i) + NUM);
            used[i] = true;
            dfs(picks, deli, delis, used);
            deli.removeLast();
            used[i] = false;
        }
        return delis;
    }




}
