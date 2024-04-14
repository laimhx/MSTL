package PickandDelivery.PDPLabel;

import Common.*;
import static AlgoRun.algoParam.*;

import java.util.*;


//Dynamic algorithm with implicit enumeration on delivery
public class SP_label {
    public Instance instance; //the current instance
    public ArrayList<Integer>[] nodeLabels; //nodeLabels[i] = position of labels in QUE with current node i for checking dominance
    public int[] checkDom; //checkDom[i] = start position of nodeLabels[i] for checking dominance (include pick and deli)
    public ArrayList<Label> QUE; //label heap structure: no deletion
    public TreeSet<Integer> HEAP;  //set of extendable labels: minimum-cost first
    public HashSet<Label> alreadyLabel; //all feasible labels generated previously
    public HashSet<Label> negRoute; //record of negative-cost labels generated so far
    public ArrayList<Label> negLabels; //new labels with reduced cost < 0
    public int negNum;  //total number of new labels with negative cost
    public int negPre; //number of previous labels with negative cost after updating dual prices
    public int domlabels; //number of dominated labels

    public int NUM;
    public int maxB;
    public double[] pi;  //dual price
    public double eps;
    public boolean first;
    public boolean disp;

    public SP_label(Instance instance,boolean first,boolean disp,int maxB,double[] pi,HashSet<Label> alreadyLabels,HashSet<Label> negRoutes) {
        this.instance = instance;  
        this.NUM = instance.NUM;
        this.maxB = maxB;
        this.pi = pi;
        this.first = first;
        this.disp = disp;
        this.alreadyLabel = alreadyLabels;
        this.negRoute = negRoutes;
    }
    

    public int run() {
        eps = 1e-5;

        //Set the empty sets
        QUE = new ArrayList<>();
        HEAP = new TreeSet<Integer>(new MyLabelComparator()); //set of labels' positions in QUE to be extended
        negLabels = new ArrayList<>();
        nodeLabels = new ArrayList[2 * NUM + 1];
        for (int i = 1; i <= 2 * NUM; i++) {
            nodeLabels[i] = new ArrayList<>();
        }
        checkDom = new int[2 * NUM + 1]; //check[i] = index of node for checking dominance

        //reset the number to be 0
        negNum = 0; negPre = 0; domlabels = 0;

        //Initialization: no negative cost labels
        initialLabels(); //check any previous labels with negative costs: numPre
        if (negPre > 0 && disp) {
            System.out.printf("Labelling started: Que %d, Heap %d, Pre %d, Neg %d %n",
                    QUE.size(), HEAP.size(), negPre, negRoute.size());
        }
        negNum += negPre;

        //Stop when MAXL routes are found with negative cost
        long s1 = System.currentTimeMillis();
        double tt = 0;        
        while (!HEAP.isEmpty() && negNum < MAXL && tt < MAXLT) {
            int lb = HEAP.pollFirst(); //index of label in QUE
            Label current = QUE.get(lb);

            domCheck(current);
            if (!current.dominated) {
                extending(current,lb); //expanding negRoute
            }

            long e1 = System.currentTimeMillis();
            tt = (e1 - s1)/1000.0;
        }

        if (tt >= MAXLT || disp) { // HEAP.isEmpty()
            System.out.printf("Labelling stopped: QUE %d, HEAP %d, routes %d, %d dominance, time %.2f seconds %n",
                    QUE.size(), HEAP.size(), negNum, domlabels, tt);
        }

        //Record the routes from completed labels
        int mstl2 = 0, mstl3 = 0, mstl4 = 0, mstl5 = 0;
        for (Label label : negLabels) {
            if (label.Picks.size() == 2) {
                mstl2 ++;
            }
            if (label.Picks.size() == 3) {
                mstl3 ++;
            }
            if (label.Picks.size() == 4) {
                mstl4 ++;
            }
            if (label.Picks.size() >= 5) {
                mstl5 ++;
            }
        }

        if (disp) {
            System.out.printf("New routes generated from labels: 2-, 3-, 4-, 5M: %d, %d, %d, %d %n",
                    mstl2, mstl3, mstl4, mstl5);
        }

        //clear data
        QUE.clear();
        HEAP.clear();
        negLabels.clear();
        Arrays.fill(nodeLabels,null);
        Arrays.fill(checkDom,0);

        //end
        return negNum;
    }


    public void initialLabels() {        
        //Case: the first round of CG
        if (first) {
            for (int i = 1; i <= NUM; i++) {
                //create an empty label
                Label initial = new Label();

                //label features
                initial.firstPickUpNode = i;
                initial.curNode = i;
                Customer cus = instance.AllCustomers.get(i);
                double startTime = cus.early - instance.Graph[0][i].duration; //vehicle departure time from depot
                initial.StartServeTime = startTime + instance.Graph[0][i].duration; //start service time at the current node

                initial.Picks = new ArrayList<>();
                initial.Picks.add(i);
                initial.load = cus.demand;
                initial.accuDis = 0;
                initial.reachNum = 1;

                //Cost features
                initial.nodeCost = - pi[i]; //stop-off and dual price
                initial.lineEnd = new double[1];
                initial.lineEnd[0] = instance.Graph[i][i + NUM].distance * (instance.Rate[i][i + NUM] - instance.detour);
                initial.linedetour = instance.Graph[i][i + NUM].distance * instance.Rate[i][i + NUM];

                //set the start dominance checking position for all labels in set nodeLabels[i]
                initial.dominated = false;
                alreadyLabel.add(initial);

                //store the labels
                int nq = QUE.size();
                QUE.add(initial);
                checkDom[initial.curNode] = 0;

                //store the position of label in the QUE for node i
                HEAP.add(nq);
                nodeLabels[initial.curNode].add(nq);
            }

        } else {
            //Case: later round of CG
            for (Label lab : alreadyLabel) {
                //update node flow cost
                lab.nodeCost = (2*lab.Picks.size() - 2) * instance.stop + lab.accuDis * instance.detour;
                for (int c : lab.Picks) {
                    lab.nodeCost -= pi[c];
                }

                double cost = lab.nodeCost + lab.linedetour;
                lab.routeCost = cost;
                for(int i : lab.Picks){
                    lab.routeCost += pi[i];
                }
                /*Adds the specified element to this set if it is not already present.
                true if this set did not already contain the specified element*/
                if (cost <= -eps && negRoute.add(lab)) {
                    negLabels.add(lab);
                    negPre ++;
                }

                //restore the previous labels
                if (lab.Picks.size() < maxB && lab.reachNum > 0) { //cost >= 0 &&
                    lab.dominated = false; //reset dominance information
                    int nq = QUE.size();
                    QUE.add(lab);
                    checkDom[lab.curNode] = 0;

                    //store the position of label in the QUE for node i
                    HEAP.add(nq);
                    nodeLabels[lab.curNode].add(nq);
                }

            }

        }

    }


    //Label extension: implicit enumerating deliveries
    public void extending(Label parent, int lb) {
        ArrayList<Integer> Reachs = setReachable(parent);
        parent.reachNum = Reachs.size();

        if (!Reachs.isEmpty()) {
            for (int j : Reachs) {
                // Check the maximum limit on labels
                if (negNum > MAXL)
                    break;

                //The new label
                Label extend = newLabel(parent, j);

                //Check feasibility
                if (extend.linedetour == Double.MAX_VALUE)
                    continue;

                //Add a new label
                alreadyLabel.add(extend);

                //Negative-cost routes
                double cost = extend.nodeCost + extend.linedetour;
                extend.routeCost = cost;
                for(int i : extend.Picks){
                    extend.routeCost += pi[i];
                }
                if (cost <= -eps && negRoute.add(extend)) {
                    negLabels.add(extend);
                    negNum ++;
                }

                //check if the label is a completed one
                if (extend.Picks.size() < maxB && extend.reachNum > 0) { //cost >= 0 &&
                    //Add the label into QUE structure
                    int nq = QUE.size();
                    QUE.add(extend);

                    /*(TreeSet) boolean add(E e): Adds the specified element to this set if it is not already present.
                    More formally, adds the specified element e to this set if the set contains no element e2 such that
                    (e==null, e2==null, or e.equals(e2)).
                    If this set already contains the element, the call leaves the set unchanged and returns false.*/
                    if (!HEAP.add((Integer) nq)) { //i.e., HEAP.add(nq) is false
                        //this label is already added to existing one and then not usable
                        QUE.get(nq).dominated = true;
                    } else {
                        //this label is successfully added into heap
                        nodeLabels[extend.curNode].add(nq);
                    }

                }

            }

        } else {
            //no extendable and remove from heap
            HEAP.remove((Integer) lb);
            nodeLabels[parent.curNode].remove((Integer) lb);
        }

    }


    //当前访问节点为j,标签信息更正 ,只扩展 pick 节点
    public Label newLabel(Label parent, int j) {
        Label extend = new Label();
        extend.firstPickUpNode = parent.firstPickUpNode;
        extend.curNode = j;

        Customer cus = instance.AllCustomers.get(j);
        extend.StartServeTime = Math.max(cus.early, parent.StartServeTime +
                instance.AllCustomers.get(parent.curNode).serve/60 +
                instance.Graph[parent.curNode][j].duration);

        extend.Picks = new ArrayList<>(parent.Picks);
        extend.Picks.add(j); //temporarily only add the pick
        extend.load = parent.load + cus.demand;
        extend.accuDis = parent.accuDis + instance.Graph[parent.curNode][j].distance;

        //record stop-off, dual price, and pickup detour costs
        extend.nodeCost = parent.nodeCost + 2 * instance.stop - pi[j] +
                instance.detour * instance.Graph[parent.curNode][j].distance;
        extend.reachNum = 1;

        //lineDetour(extend); //an optimal delivery sequence
        setlineDetour(extend); //a feasible delivery sequence

        return extend;
    }


    //Set the unreachable nodes for labels
    public ArrayList<Integer> setReachable(Label label) {
        ArrayList<Integer> Reachs = new ArrayList<>();

        if (label.Picks.size() < maxB) {
            //search for nodes to extend
            for (int j = 1; j <= NUM; j++) {
                if (label.Picks.contains(j)) {
                    continue;
                }

                //load capacity constraint
                if (label.load + instance.AllCustomers.get(j).demand > capacity) {

                    continue;
                }

                //bundling radius limit
                if (instance.Graph[label.curNode][j].distance > instance.radius) {
                    continue;
                }

                //time windows constraint based on triangle inequalities
                if (label.StartServeTime + instance.AllCustomers.get(label.curNode).serve/60 +
                        instance.Graph[label.curNode][j].duration > instance.AllCustomers.get(j).last) {
                    continue;
                }

                //add the extendable nodes
                Reachs.add(j);
            }

        }

        return Reachs;
    }


    //Search a feasible delivery sequence
    //update the label: lineCost, detourCost, time, route
    public void setlineDetour(Label label){
        int ordNum = label.Picks.size();
        int first, last;

        first = label.firstPickUpNode;
        Customer cus0 = instance.AllCustomers.get(first);
        double startTime = cus0.early - instance.Graph[0][first].duration;  //vehicle departure time from depot

        //record the LH cost
        if (instance.LIFO) {
            label.lineEnd = new double[1];//the line-haul cost for each end node
            last = first + NUM;
            label.lineEnd[0] = instance.Graph[first][last].distance * (instance.Rate[first][last] - instance.detour); //default value
        } else {
            label.lineEnd = new double[ordNum];//the line-haul cost for each end node
            for (int i = 0; i < ordNum; i++) {
                last = label.Picks.get(i) + NUM;
                label.lineEnd[i] = instance.Graph[first][last].distance * (instance.Rate[first][last] - instance.detour); //default value
            }
        }

        // Enumerate deliveries
        HashSet<ArrayList<Integer>> delis = new HashSet<>();
        if (instance.LIFO) {
            // Case: delivery sequence is reversed to pickup sequence
            ArrayList<Integer> deli = new ArrayList<>();
            for(int i = ordNum-1; i >= 0; i--)
                deli.add(label.Picks.get(i) + NUM);
            delis.add(deli);
        } else {
            // Case: enumerate the optimal delivery sequences
            delis = dfs(label.Picks,new ArrayList<>(),new HashSet<>(),new boolean[label.Picks.size()]);
        }

        // Check deliver feasibility
        label.linedetour = Double.MAX_VALUE;
        for (ArrayList<Integer> deli : delis) {
            double serveTime = label.StartServeTime; //at the current pick node
            double dist = 0;
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
            if (serveTime == Double.MAX_VALUE) { //infeasible label
                continue; //evaluate the next sequence
            }

            //hours of service limit
            last = deli.getLast();
            double time = (serveTime + instance.AllCustomers.get(last).serve / 60 +
                    instance.Graph[last][2 * NUM + 1].duration) - startTime;
            if (time <= timelimit) {
                //stop-off and dual price are fixed by the picks
                double lh = instance.Graph[first][last].distance * (instance.Rate[first][last] - instance.detour);
                label.linedetour = lh + instance.detour * dist;
                break; //stop whenever feasible
            }
        }

    }


    //Search the optimal delivery sequence
    public void lineDetour(Label label){
        int ordNum = label.Picks.size();

        int first = label.firstPickUpNode;
        Customer cus0 = instance.AllCustomers.get(first);
        double startTime = cus0.early - instance.Graph[0][first].duration;  //vehicle departure time from depot

        //record the LH cost
        if (instance.LIFO) {
            label.lineEnd = new double[1];
            label.lineEnd[0] = Double.MAX_VALUE; //default value
        } else {
            label.lineEnd = new double[ordNum];
            for (int i = 0; i < ordNum; i++) {
                label.lineEnd[i] = Double.MAX_VALUE; //default value
            }
        }

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
                int id = last - NUM; //the id of pick
                int k = label.Picks.indexOf(id); //index of id in picks

                double lh = instance.Graph[first][last].distance * (instance.Rate[first][last] - instance.detour);
                label.lineEnd[k] = lh;

                //stop-off and dual price are fixed by the picks
                double costs = lh + instance.detour * dist;
                if (costs < label.linedetour) {
                    label.linedetour = costs;
                }
            }
        }

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


    //Check dominance between label1 and label2:
    //Dominance among the labels with the same current node without repetition
    public void domCheck(Label current) {
        int l1, l2;
        boolean timedom, DTdom, pathdom, LHdom;
        Label label1, label2;
        ArrayList<Integer> cleaning = new ArrayList<Integer>(); //set of dominated nodes

        //if checkDom[current.curNode]=0, then start checking from the first label
        for (int i = checkDom[current.curNode]; i < nodeLabels[current.curNode].size(); i++) {
            for (int j = 0; j < i; j++) {
                l1 = nodeLabels[current.curNode].get(i);
                l2 = nodeLabels[current.curNode].get(j);
                label1 = QUE.get(l1);
                label2 = QUE.get(l2);

                //condition 1: first pickUp node and current node
                if (label1.firstPickUpNode != label2.firstPickUpNode) { //first pick node
                    continue;
                }
                if (label1.curNode != label2.curNode) { //first pick node
                    continue;
                }

                if (!(label1.dominated || label2.dominated)) {
                    //Case: Check if label1 dominates label2
                    //Condition 2: time
                    timedom = (label1.StartServeTime <= label2.StartServeTime);

                    pathdom = false; DTdom = false;
                    if (timedom) {
                        //condition 3: label1.onboardOrders <= label2.onboardOrders
                        //label1.onboardOrders[k]=0,label2.onboardOrders[k]=1;
                        //label1.onboardOrders[k]=1,label2.onboardOrders[k]=1;
                        pathdom = true;
                        int k = 1;
                        while (pathdom && k <= NUM) {
                            pathdom = (!label1.Picks.contains(k) || label2.Picks.contains(k));// break if false
                            k ++;
                        }
                        //condition 4: flow cost
                        DTdom = (label1.nodeCost <= label2.nodeCost);
                    }

                    //condition 5: line-haul costs
                    LHdom = true;
                    if (!instance.LIFO && pathdom && DTdom) {
                        LHdom = checkLine(label1, label2);
                    }

                    if (pathdom && DTdom && LHdom) {
                        QUE.get(l2).dominated = true;
                        HEAP.remove((Integer) l2);
                        cleaning.add(l2);
                        domlabels ++;
                    }

                    if (QUE.get(l2).dominated)
                        continue;

                    //-------------------------------------------------------------------------------------//
                    //Case: Check if label2 dominates label1
                    //Condition 2: time
                    timedom = (label2.StartServeTime <= label1.StartServeTime);

                    pathdom = false; DTdom = false;
                    if (timedom) {
                        //condition 3: label2.onboardOrders <= label1.onboardOrders
                        pathdom = true;
                        int k = 1;
                        while (pathdom && k <= NUM) {
                            pathdom = (!label2.Picks.contains(k) || label1.Picks.contains(k));// break if false
                            k ++;
                        }
                        //condition 4: flow cost
                        DTdom = (label2.nodeCost <= label1.nodeCost);
                    }

                    //condition 5: line-haul costs
                    LHdom = true;
                    if (!instance.LIFO && pathdom && DTdom) {
                        LHdom = checkLine(label2, label1);
                    }

                    if (pathdom && DTdom && LHdom) {
                        QUE.get(l1).dominated = true;
                        HEAP.remove((Integer) l1);
                        cleaning.add(l1);
                        domlabels ++;

                        //restart from next i by setting j>=i
                        j = nodeLabels[current.curNode].size();
                    }

                }

            }

        }

        //Delete the dominated labels in set nodeLabels[i]
        for (Integer lb : cleaning){
            nodeLabels[current.curNode].remove((Integer) lb);
        }

        //Update the starting position for checking dominance
        checkDom[current.curNode] = nodeLabels[current.curNode].size();
    }


    //Check LS >dom LH only in NON-LIFO case
    public boolean checkLine(Label LS, Label LH) {
        //the maximum LH in label LS
        double[] lhs = Arrays.copyOf(LS.lineEnd, LS.lineEnd.length);
        Arrays.sort(lhs);
        double maxLS = lhs[lhs.length-1];

        //the minimum LH in label LH while not in LS
        double minLH = Double.MAX_VALUE;
        for (int i = 0; i < LH.Picks.size(); i++) {
            int id = LH.Picks.get(i);
            if (!LS.Picks.contains(id) && LH.lineEnd[i] < minLH) {
                minLH = LH.lineEnd[i];
            }
        }

        //Dummy case: minLH = Double.MAX_VALUE
        return (minLH - maxLS >= eps);
    }


    class MyLabelComparator implements Comparator<Integer> {
        // treeSet is an ordered list
        // to maintain the order, we need to define a comparator: cost is the main criterion
        public int compare(Integer a, Integer b) {
            Label A = QUE.get(a);
            Label B = QUE.get(b);

            // Be careful! When the comparator returns 0, it means that the two labels are considered EXACTLY the same ones!
            // This comparator is not only used to sort the lists!  When adding to the list, a value of 0 => not added!!!!!
            if (A.linedetour + A.nodeCost - B.linedetour - B.nodeCost < -eps)
                return -1;
            else if (A.linedetour + A.nodeCost - B.linedetour - B.nodeCost > eps)
                return 1;
            else {
                if (A.curNode == B.curNode) {
                    if (A.StartServeTime - B.StartServeTime < -eps)
                        return -1;
                    else if (A.StartServeTime - B.StartServeTime > eps)
                        return 1;
                    else if (A.load - B.load < -eps)
                        return -1;
                    else if (A.load - B.load > eps)
                        return 1;
                    else {
                        int i = 1;
                        while (i <= NUM) {
                            if (A.Picks.contains(i) != B.Picks.contains(i)) {
                                if (A.Picks.contains(i)) //longer label first???
                                    return -1;
                                else
                                    return 1;
                            }
                            i++;
                        }
                        return 0;
                    }
                } else if (A.curNode > B.curNode)
                    return 1;
                else
                    return -1;
            }
        }
    }


}
