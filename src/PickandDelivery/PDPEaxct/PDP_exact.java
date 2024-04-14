package PickandDelivery.PDPEaxct;

import Common.*;

import java.text.ParseException;
import java.util.ArrayList;
import java.util.HashMap;

//import gurobi.*;
import com.gurobi.gurobi.*;

import static AlgoRun.algoParam.*;

public class PDP_exact {
    public Instance instance;
    public double[][] Arc;
    public double[] demand;
    public double[] early;
    public double[] late;
    public double[] service;
    public double[] LTLcost;
    public int NUM;
    public int numK;
    public int nvar; //number of variables
    public int nconstr; //number of constraints
    public int feasb;
    public double optCost;
    public double maxTime;
    public double eps;
    public String solOpt;


    public PDP_exact(Instance instance) {
        this.instance = instance;
        this.NUM = instance.NUM;
        this.Arc = instance.Arc;
    }


    public void run() throws ParseException {
        //start the algorithm
        long startTime = System.currentTimeMillis();
        numK = (int) Math.ceil(0.5*NUM);
        maxTime = 43200; //time limit: 6 hours = 21600 sec; 12 hours = 43200.0 sec
        eps = 1e-5;

        //Process the shipping network
        shipNetwork();

        //Solve the original MSP model
        instance.routesExact = solveModel();

        //Algorithm stopped
        long endTime = System.currentTimeMillis();
        double tt = (endTime - startTime)/1000.0/60;
        System.out.printf("------- Exact algorithm is completed with %.4f min: %d vars, %d constrs, feasibility %d, " +
                        "status %s. -------%n", tt, nvar, nconstr, feasb, solOpt);

        //Record the results
        instance.numVar = nvar;
        instance.numConstr = nconstr;
        instance.feasb_exact = feasb;
        instance.TIME_exact = tt;

        //clear temp data
    }


    //The shipping network
    public void shipNetwork() {
        //pick point index: 0, 1, 2, ..., NUM;
        //delivery point index: NUM+1, ..., 2*NUM, 2*NUM+1

        //The order data
        demand = new double[2*NUM+2];
        early = new double[2*NUM+2];
        late = new double[2*NUM+2];
        service = new double[2*NUM+2];
        LTLcost = new double[NUM+1];
        for (int i = 0; i <= 2*NUM+1; i++) {
            Customer customer = instance.AllCustomers.get(i);

            if (i >= 1 && i <= 2*NUM) {
                demand[i] = customer.demand;
                early[i] = customer.early;
                late[i] = customer.last;
                service[i] = customer.serve/60;
            } else { //node 0 and 2n+1
                demand[i] = 0;
                early[i] = 0;
                late[i] = timehorizon;
                service[i] = 0;
            }

            if (i >= NUM+1 && i <= 2*NUM && demand[i] > 0) {
                System.out.printf("###### Demand of delivery node %d is NOT negative: %.2f###### %n",
                        i, demand[i]);
            }

            if (i >= 1 && i <= NUM) {
                LTLcost[i] = customer.singleCost;
            }
        }
    }


    //Solve the PDP model by Gurobi
    public HashMap<Integer, Route> solveModel() {
        HashMap<Integer,Route> routeSol = new HashMap<>();
        optCost = 0;
        nvar = 0; nconstr = 0;

        try {
            //Set up the model
            GRBEnv env = new GRBEnv();
            GRBModel model = new GRBModel(env);
            model.set(GRB.StringAttr.ModelName, "exactModel");

            String str;
            double lb, ub, cx;
            double bigM = 1e3;
            GRBLinExpr lhs, rhs, expr;

             /*Index the variables: x_{ij}^{k}, y_{ij}^{k}, z_{i}, t_{i}^{k}, q_{i}^{k}
             x_{ij}^{1}: x_{0,1}^{1}, ..., x_{n,2n+1}^{1}
             y_{ij}^{1}: y_{1,1}^{1}, ..., y_{2n,2n}^{1}
             z_{i}: z_{1}, z_{2}, ..., z_{n}
             t_{i}^{1}: t_{1}^{1}, ..., t_{1}^{n}
             q_{i}^{1}: q_{1}^{1}, ..., q_{1}^{n}
             */

            //variable x_{ij}^{k}: 0 <= i <= 2*NUM+1, 0 <= j <= 2*NUM+1, 0 <= k <= numK-1
            GRBVar[][][] X = new GRBVar[2*NUM+2][2*NUM+2][numK];
            for (int i = 0; i <= 2*NUM+1; i++) {
                for (int j = 0; j <= 2*NUM+1; j++) {
                    for (int k = 0; k < numK; k++) {
                        str = "X_" + i + "_" + j + "_" + k;
                        X[i][j][k] = model.addVar(0.0, Arc[i][j], 0.0, GRB.BINARY, str);
                        nvar ++;
                    }
                }
            }

            //variable y_{ij}^{k}: 1 <= i <= NUM, 1 <= j <= NUM, 0 <= k <= numK-1
            GRBVar[][][] Y = new GRBVar[NUM+1][NUM+1][numK];
            for (int i = 0; i <= NUM; i++) {
                for (int j = 0; j <= NUM; j++) {
                    for (int k = 0; k < numK; k++) {
                        //variable y_{ij}^{k}
                        str = "Y_" + i + "_" + j + "_" + k;

                        ub = 0.0; //set y_{0j}^{k} or y_{i0}^{k} to be 0
                        if (i > 0 && j > 0) {
                            ub = 1.0;
                        }

                        Y[i][j][k] = model.addVar(0.0, ub, 0.0, GRB.BINARY, str);
                        nvar ++;
                    }
                }
            }

            //variable z_{i}: 1 <= i <= NUM
            GRBVar[] Z = new GRBVar[NUM+1];
            str = "Z_" + 0;
            Z[0] = model.addVar(0.0, 0.0, 0.0, GRB.BINARY, str);
            for (int i = 1; i <= NUM; i++) {
                str = "Z_" + i;
                Z[i] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, str);
                nvar ++;
            }

            //variable t_{i}^{k}, q_{i}^{k}: 0 <= i <= 2*NUM+1, 0 <= k <= numK-1
            GRBVar[][] T = new GRBVar[2*NUM+2][numK];
            GRBVar[][] Q = new GRBVar[2*NUM+2][numK];
            for (int k = 0; k < numK; k++) {
                for (int i = 0; i <= 2*NUM+1; i++) {
                    str = "T_" + i + "_" + k;
                    T[i][k] = model.addVar(0.0, timehorizon, 0.0, GRB.CONTINUOUS, str);
                    nvar ++;

                    str = "Q_" + i + "_" + k;
                    Q[i][k] = model.addVar(0.0, capacity, 0.0, GRB.CONTINUOUS, str);
                    nvar ++;
                }
            }

            //Constraints: mode selection sum_{k}sum_{1<=j<=2n}x_{ij}^k + z_i =1
            for (int i = 1; i <= NUM; i++) {
                lhs = new GRBLinExpr();
                for (int j = 1; j <= 2*NUM; j++) {
                    for (int k = 0; k < numK; k++) {
                        lhs.addTerm(Arc[i][j], X[i][j][k]);
                    }
                }
                lhs.addTerm(1.0, Z[i]);
                model.addConstr(lhs, GRB.EQUAL, 1, "assign" + "_" + i);
                nconstr ++;
            }

            //Constraints: same vehicle for pickup and delivery
            //sum_{j}x_{ij}^{k} = sum_{j}x_{n+i,j}^{k}
            for (int i = 1; i <= NUM; i++) {
                for (int k = 0; k < numK; k++) {
                    lhs = new GRBLinExpr();
                    for (int j = 1; j <= 2*NUM; j++) {
                        lhs.addTerm(Arc[i][j], X[i][j][k]);
                    }

                    rhs = new GRBLinExpr();
                    for (int j = NUM+1; j <= 2*NUM+1; j++) {
                        rhs.addTerm(Arc[NUM+i][j], X[NUM+i][j][k]);
                    }

                    model.addConstr(lhs, GRB.EQUAL, rhs, "select" + "_" + i + "_" + k);
                    nconstr ++;
                }
            }

            //Constraints: vehicle flow balance at ends
            for (int k = 0; k < numK; k++) {
                //Start node 0: sum_{j<=n}x_{0j}^{k} + x_{0,2n+1}^{k} = 1
                lhs = new GRBLinExpr();
                for (int j = 1; j <= NUM; j++) {
                    lhs.addTerm(1.0, X[0][j][k]);
                }
                lhs.addTerm(1.0, X[0][2*NUM+1][k]);
                model.addConstr(lhs, GRB.EQUAL, 1, "flow" + "_0_" + k);
                nconstr ++;

                //End node 2n+1: x_{0,2n+1}^{k} + sum_{i>n}x_{i,2n+1}^{k} = 1
                lhs = new GRBLinExpr();
                lhs.addTerm(1.0, X[0][2*NUM+1][k]);
                for (int j = NUM+1; j <= 2*NUM; j++) {
                    lhs.addTerm(1.0, X[j][2*NUM+1][k]);
                }
                model.addConstr(lhs, GRB.EQUAL, 1, "flow" + "_" + (2 * NUM + 1) + "_" + k);
                nconstr ++;
            }

            //Flow balance at intermediate node i: sum_{j}x_{ij}^{k} = sum_{j}x_{ji}^{k}
            for (int k = 0; k < numK; k++) {
                for (int i = 1; i <= 2*NUM; i++) {
                    //outgoing flows
                    lhs = new GRBLinExpr();
                    for (int j = 1; j <= 2*NUM + 1; j++){
                        lhs.addTerm(Arc[i][j], X[i][j][k]);
                    }

                    //incoming flows
                    rhs = new GRBLinExpr();
                    for (int j = 0; j <= 2*NUM; j++){
                        rhs.addTerm(Arc[j][i], X[j][i][k]);
                    }

                    model.addConstr(lhs, GRB.EQUAL, rhs, "flow" + "_" + i + "_" + k);
                    nconstr ++;
                }
            }

            //Constraints: the first pickup and last delivery points
            //first pickup: y_{ij}^{k} <= x_{0i}^{k}
            for (int k = 0; k < numK; k++) {
                for (int i = 1; i <= NUM; i++) {
                    for (int j = 1; j <= NUM; j++) {
                        lhs = new GRBLinExpr();
                        lhs.addTerm(1.0, Y[i][j][k]);
                        lhs.addTerm(-1.0, X[0][i][k]);

                        model.addConstr(lhs, GRB.LESS_EQUAL, 0, "first" + "_" + i + "_" + j + "_" + k);
                        nconstr ++;
                    }
                }
            }

            //last delivery: y_{ij}^{k} <= x_{n+j,2n+1}^{k}
            for (int k = 0; k < numK; k++) {
                for (int j = 1; j <= NUM; j++) {
                    for (int i = 1; i <= NUM; i++) {
                        lhs = new GRBLinExpr();
                        lhs.addTerm(1.0, Y[i][j][k]);
                        lhs.addTerm(-1.0, X[NUM+j][2*NUM+1][k]);

                        model.addConstr(lhs, GRB.LESS_EQUAL, 0, "last" + "_" + i + "_" + j + "_" + k);
                        nconstr ++;
                    }
                }
            }

            //first pickup i: x_{0i}^{k} = sum_{j} y_{ij}^{k}
            for (int k = 0; k < numK; k++) {
                for (int i = 1; i <= NUM; i++) {
                    lhs = new GRBLinExpr();
                    lhs.addTerm(1.0, X[0][i][k]);

                    rhs = new GRBLinExpr();
                    for (int j = 1; j <= NUM; j++) {
                        rhs.addTerm(1.0, Y[i][j][k]);
                    }

                    model.addConstr(lhs, GRB.EQUAL, rhs, "first" + "_" + i + "_" + k);
                    nconstr ++;
                }
            }

            //last delivery j: x_{n+j,2n+1}^{k} = sum_{i} y_{ij}^{k}
            for (int k = 0; k < numK; k++) {
                for (int j = 1; j <= NUM; j++) {
                    lhs = new GRBLinExpr();
                    lhs.addTerm(1.0, X[NUM+j][2*NUM+1][k]);

                    rhs = new GRBLinExpr();
                    for (int i = 1; i <= NUM; i++) {
                        rhs.addTerm(1.0, Y[i][j][k]);
                    }

                    model.addConstr(lhs, GRB.EQUAL, rhs, "last" + "_" + j + "_" + k);
                    nconstr ++;
                }
            }

            //first-last pair: sum_{i}sum_{j}y_{ij}^{k} <= 1
            for (int k = 0; k < numK; k++) {
                lhs = new GRBLinExpr();
                for (int i = 1; i <= NUM; i++) {
                    for (int j = 1; j <= NUM; j++) {
                        lhs.addTerm(1.0, Y[i][j][k]);
                    }
                }
                model.addConstr(lhs, GRB.LESS_EQUAL, 1.0, "lane" + "_" + String.valueOf(k));
                nconstr ++;
            }

            //Constraints: tracking of time
            for (int k = 0; k < numK; k++) {
                //t_{2n+1}^{k} - t_{0}^{k} <= timelimit
                lhs = new GRBLinExpr();
                lhs.addTerm(-1.0, T[0][k]);
                lhs.addTerm(1.0, T[2*NUM+1][k]);
                model.addConstr(lhs, GRB.LESS_EQUAL, timelimit, "timelimit" + "_" + k);
                nconstr ++;

                //t_{0}^{k} >= (e_{i} - a_{0i} -s_{0}) x_{0i}^{k}
                //i.e., (e_{i} - a_{0i} -s_{0}) x_{0i}^{k} - t_{0}^{k} <= 0
                for (int i = 1; i <= NUM; i++) {
                    lhs = new GRBLinExpr();
                    cx = early[i] - instance.Graph[0][i].duration;
                    lhs.addTerm(cx, X[0][i][k]);
                    lhs.addTerm(-1.0, T[0][k]);
                    model.addConstr(lhs, GRB.LESS_EQUAL, 0, "start_time" + "_0_" + i + "_" + k);
                    nconstr ++;
                }

                //t_{j}^{k} >= t_{i}^{k} + a_{ij} + s_{i} - M*(1-x_{ij}^{k})
                //i.e.,t_{i}^{k} - t_{j}^{k} + M*x_{ij}^{k} <= M - (a_{ij} + s_{i})
                for (int i = 0; i <= 2*NUM; i++) {
                    for (int j = 1; j <= 2*NUM+1; j++) {
                        boolean flag = (i > 0 | j < 2*NUM+1);
                        if (flag && Arc[i][j] > 0) {
                            lhs = new GRBLinExpr();
                            lhs.addTerm(1.0, T[i][k]);
                            lhs.addTerm(-1.0, T[j][k]);
                            lhs.addTerm(bigM, X[i][j][k]);

                            ub = bigM - instance.Graph[i][j].duration - service[i];
                            model.addConstr(lhs, GRB.LESS_EQUAL, ub, "arc_time" + "_" + i + "_" + j + "_" + k);
                            nconstr ++;
                        }
                        //dummy arc (0,2n+1): t_{2n+1}^{k} >= t_{0}^{k} + 0
                    }
                }

                //t_{n+i}^{k} - t_{i}^{k} >= a_{i,n+i} + s_{i}
                for (int i = 1; i <= NUM; i++) {
                    lhs = new GRBLinExpr();
                    lhs.addTerm(1.0, T[NUM+i][k]);
                    lhs.addTerm(-1.0, T[i][k]);

                    lb = instance.Graph[i][NUM+i].duration + service[i];
                    model.addConstr(lhs, GRB.GREATER_EQUAL, lb, "order_time" + "_" + i + "_" + (NUM+i) + "_" + k);
                    nconstr ++;
                }

                //time window: t_{i}^{k} >= e_{i}, t_{n+i}^{k} <= l_{i}
                for (int i = 1; i <= NUM; i++) {
                    lhs = new GRBLinExpr();
                    lhs.addTerm(1.0, T[i][k]);
                    lb = early[i];
                    model.addConstr(lhs, GRB.GREATER_EQUAL, lb, "time_window" + "_" + i + "_" + k);
                    nconstr ++;

                    lhs = new GRBLinExpr();
                    lhs.addTerm(1.0, T[NUM+i][k]);
                    ub = late[NUM+i];
                    model.addConstr(lhs, GRB.LESS_EQUAL, ub, "time_window" + "_" + (NUM+i) + "_" + k);
                    nconstr ++;
                }
            }

            //Constraints: tracking of loads
            for (int k = 0; k < numK; k++) {
                //q_{0}^{k} = 0;
                lhs = new GRBLinExpr();
                lhs.addTerm(1.0, Q[0][k]);
                model.addConstr(lhs, GRB.EQUAL, 0, "start_load" + "_0_" + k);
                nconstr ++;

                //q_{2n+1}^{k} = 0;
                lhs = new GRBLinExpr();
                lhs.addTerm(1.0, Q[2*NUM+1][k]);
                model.addConstr(lhs, GRB.EQUAL, 0, "final_load" + "_" + (2*NUM+1) + "_" + k);
                nconstr ++;

                //q_{j}^{k} >= q_{i}^{k} + w_{j} - M*(1-x_{ij}^{k})
                //i.e.,q_{i}^{k} - q_{j}^{k} + M*x_{ij}^{k} <= M - w_{j}
                for (int i = 0; i <= 2*NUM; i++) {
                    for (int j = 1; j <= 2*NUM+1; j++) {
                        boolean flag = (i > 0 | j < 2*NUM+1);
                        if (flag && Arc[i][j] > 0) {
                            lhs = new GRBLinExpr();
                            lhs.addTerm(1.0, Q[i][k]);
                            lhs.addTerm(-1.0, Q[j][k]);
                            lhs.addTerm(bigM, X[i][j][k]);

                            ub = bigM - demand[j];
                            model.addConstr(lhs, GRB.LESS_EQUAL, ub, "arc_load" + "_" + i + "_" + j + "_" + k);
                            nconstr ++;
                        }
                    }
                }
            }

            //Constraints: bundling size
            //sum_{0<=i<=n}sum_{1<=j<=n} x_{ij}^{k} <= B
            for (int k = 0; k < numK; k++) {
                for (int i = 0; i <= NUM; i++) {
                    for (int j = 1; j <= NUM; j++) {
                        lhs = new GRBLinExpr();
                        lhs.addTerm(Arc[i][j], X[i][j][k]);
                        ub = instance.maxOrders;
                        model.addConstr(lhs, GRB.LESS_EQUAL, ub, "bundling" + "_" + k);
                        nconstr ++;
                    }
                }
            }

            //Set objective function
            expr = new GRBLinExpr();
            for (int k = 0; k < numK; k++) {
                //coefficients for variable y_{ij}^{k}
                for (int i = 1; i <= NUM; i++) {
                    for (int j = 1; j <= NUM; j++) {
                        cx = (instance.Rate[i][NUM+j] - instance.detour)*instance.Graph[i][NUM+j].distance
                                - instance.stop;
                        expr.addTerm(cx, Y[i][j][k]);
                    }
                }

                //coefficients for variable x_{ij}^{k}
                for (int i = 0; i <= 2*NUM; i++) {
                    for (int j = 1; j <= 2*NUM+1; j++) {
                        if (i > 0 && j < 2*NUM+1 && Arc[i][j] > 0) {
                            cx = instance.stop + instance.detour*instance.Graph[i][j].distance;
                            expr.addTerm(cx, X[i][j][k]);
                        }
                    }
                }
            }

            //coefficients for variable z_{i}
            for (int i = 1; i <= NUM; i++) {
                cx = LTLcost[i];
                expr.addTerm(cx, Z[i]);
            }

            /*
            //set the starting solution
            for (int i = 0; i <= 2*NUM+1; i++) {
                for (int j = 0; j <= 2*NUM+1; j++) {
                    for (int k = 0; k < numK; k++) {
                        if (i == 0 && j == 2*NUM+1) {
                            X[i][j][k].set(GRB.DoubleAttr.Start, 1);
                        } else {
                            X[i][j][k].set(GRB.DoubleAttr.Start, 0);
                        }
                    }
                }
            }

            for (int i = 0; i <= NUM; i++) {
                for (int j = 0; j <= NUM; j++) {
                    for (int k = 0; k < numK; k++) {
                        Y[i][j][k].set(GRB.DoubleAttr.Start, 0);
                    }
                }
            }

            Z[0].set(GRB.DoubleAttr.Start, 0);
            for (int i = 1; i <= NUM; i++) {
                Z[i].set(GRB.DoubleAttr.Start, 1);
            }

            for (int k = 0; k < numK; k++) {
                for (int i = 0; i <= 2*NUM+1; i++) {
                    if (i == 0 || i == 2*NUM+1) {
                        T[i][k].set(GRB.DoubleAttr.Start, 0);
                    }

                    if (i > 0 && i <= NUM) {
                        T[i][k].set(GRB.DoubleAttr.Start, early[i]);
                    }
                    if (i > NUM && i <= 2*NUM) {
                        T[i][k].set(GRB.DoubleAttr.Start, late[i]);
                    }

                    Q[i][k].set(GRB.DoubleAttr.Start, 0);
                }
            }
            */

            //Minimize the costs
            model.setObjective(expr, GRB.MINIMIZE);

            //The optimization parameters
            model.set(GRB.IntParam.OutputFlag, 0); //no log output
            model.set(GRB.DoubleParam.TimeLimit, maxTime);

            //Solve the model
            model.optimize();

            //Check validity of solution
            int status = model.get(GRB.IntAttr.Status);
            feasb = 1;
            if(status == GRB.Status.INF_OR_UNBD || status == GRB.Status.INFEASIBLE){
                feasb = 0;
                System.exit(0);
            }

            if(status <= 2){
                solOpt = "OPTIMAL";
            } else if(status <= 5){
                solOpt = "INFEASIBLE";
            } else {
                solOpt = "FEASIBLE";
            }
            System.out.printf("###### The exact problem was stopped with status %d: %s ###### %n", status, solOpt);

            //The optimum solution
            optCost = model.get(GRB.DoubleAttr.ObjVal);

            double[][][] varX = new double[2*NUM+2][2*NUM+2][numK];
            double[][][] varY = new double[NUM+1][NUM+1][numK];
            double[] varZ = new double[NUM+1];
            for (int k = 0; k < numK; k++) {
                //variable x_{ij}^{k}
                for (int i = 0; i <= 2*NUM+1; i++) {
                    for (int j = 0; j <= 2*NUM+1; j++) {
                        double v = (1-X[i][j][k].get(GRB.DoubleAttr.X)) * X[i][j][k].get(GRB.DoubleAttr.X);
                        if (v > eps) {
                            System.err.printf("###### The exact solution of X is highly fractional: %.4f ###### %n", v);
                        }
                        if (Math.abs(X[i][j][k].get(GRB.DoubleAttr.X) - 1) <= eps) {
                            varX[i][j][k] = 1;
                        } else {
                            varX[i][j][k] = 0;
                        }
                    }
                }

                //variable y_{ij}^{k}
                for (int i = 0; i <= NUM; i++) {
                    for (int j = 0; j <= NUM; j++) {
                        double v = (1-Y[i][j][k].get(GRB.DoubleAttr.X)) * Y[i][j][k].get(GRB.DoubleAttr.X);
                        if (v > eps) {
                            System.err.printf("###### The exact solution of Y is highly fractional: %.4f ###### %n", v);
                        }

                        if (Math.abs(Y[i][j][k].get(GRB.DoubleAttr.X) - 1) <= eps) {
                            varY[i][j][k] = 1;
                        } else {
                            varY[i][j][k] = 0;
                        }
                    }
                }
            }

            //variable z_{i}
            for (int i = 0; i <= NUM; i++) {
                double v = (1-Z[i].get(GRB.DoubleAttr.X)) * Z[i].get(GRB.DoubleAttr.X);
                if (v > eps) {
                    System.err.printf("###### The exact solution of Z is highly fractional: %.4f ###### %n", v);
                }
                if (Math.abs(Z[i].get(GRB.DoubleAttr.X) - 1) <= eps) {
                    varZ[i] = 1;
                } else {
                    varZ[i] = 0;
                }
            }
            System.out.println("The exact model solution is derived.");

            //Determine the vehicle routes
            if (checkSol(varX, varY, varZ)) {
                routeSol = findRoutes(varX, varZ);
            } else {
                feasb = 0;
                System.err.println("###### The exact model solution is NOT valid! ######");
            }

            //Clear the model
            model.dispose();
            env.dispose();

        } catch (GRBException e) {
            System.err.println("###### Error code: " + e.getErrorCode() + ". ######" + e.getMessage());
        }

        //result
        return routeSol;
    }


    public boolean checkSol(double[][][] X,double[][][] Y,double[] Z){
        boolean valid = true;
        int pick, deli;

        //The first pickup and last delivery must be consistent
        out: for (int k = 0; k < numK; k++) {
            if (X[0][2*NUM+1][k] == 0) {
                for (int i = 1; i <= NUM; i++) {
                    for (int j = 1; j <= NUM; j++) {
                        boolean flag = (X[0][i][k] == 0 | X[NUM+j][2*NUM+1][k] == 0);
                        if (Y[i][j][k] == 1 && flag) {
                            valid = false;
                            pick = i; deli = j;
                            System.err.printf("###### Error: first pickup %d and last delivery %d for vehicle %d are NOT consistent ###### %n",
                                    pick, deli, k);
                            break out;
                        }
                    }
                }
            }
            //System.out.printf("The route of vehicle %d is valid! %n", k);
        }

        //LTL orders cannot be bundled
        out: for (int i = 1; i <= NUM; i++) {
            if (Z[i] == 1) {
                for (int k = 0; k < numK; k++) {
                    if (X[0][i][k] == 1) {
                        valid = false;
                        System.err.printf("###### Error: order %d mode selection is NOT consistent! ###### %n", i);
                        break out;
                    }
                }
            }
        }

        System.out.println("The exact model solution is valid!");
        return valid;
    }


    //Determine the selected routes
    public HashMap<Integer, Route> findRoutes(double[][][] X,double[] Z){
        HashMap<Integer, Route> Routes = new HashMap<>();
        double cost = 0;
        int current, id;
        Route route;

        id = 0;
        for (int k = 0; k < numK; k++) {
            route = new Route();
            if (X[0][2*NUM+1][k] == 0) { //nonempty route
                //e.g., 0 --> 3 --> 5 --> 13 --> 15 --> 21 (n = 10)
                current = 0;
                while (current <= 2*NUM) {
                    route.R.add(current);
                    for (int j = 1; j <= 2*NUM+1; j++) {
                        if (j != current && X[current][j][k] == 1) {
                            current = j;
                            break;
                        }
                    }
                }
                route.R.add(2*NUM+1);
                id ++;
                Routes.put(id, route);

                if (!instance.checkRoute(route)) {
                    feasb = 0;
                    System.err.printf("###### The exact solution route is NOT feasible: vehicle %d, size %d ###### %n",
                            k, route.R.size());
                }
                cost += route.cost;
            }
        }

        for (int i = 1; i <= NUM; i++) {
            if (Z[i] == 1) {
                route = new Route();
                route.R.add(0);
                route.R.add(i);
                route.R.add(i+instance.NUM);
                route.R.add(2*instance.NUM+1);
                route.cost = LTLcost[i];

                id ++;
                Routes.put(id, route);
                cost += route.cost;
            }
        }

        //Check the objective functions
        if (Math.abs(optCost - cost) > eps) {
            System.err.printf("###### Error: objective values are NOT equal: opt %.2f, cost %.2f ###### %n",
                    optCost, cost);
        }

        //results
        return Routes;
    }



}
