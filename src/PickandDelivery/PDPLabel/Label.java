package PickandDelivery.PDPLabel;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

public class Label {
    //Label features
    ArrayList<Integer> Picks;  //the sequence of picks
    int firstPickUpNode;
    int curNode;  //the node just arrived
    double StartServeTime;  //the starting time of service at node curNode
    double accuDis;  //the accumalated pickup detour disatance from the first pick node to the last pick node;
    double linedetour;  //the current minimum line cost ( 计算比较复杂) rate * dis[firstPick][lastDeli]

    //Auxiliary information
    int reachNum; //number of reachable nodes
    double load;  //the load after visit node curNode
    double nodeCost;  //the accumulated node cost(stop-off + pi)
    double routeCost; //the actual cost of route
    double[] lineEnd; //the line-haul cost for each end node
    boolean dominated;

    public Label() {

    }


    public int getSize(){
        return Picks.size();
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;

        if (o == null || getClass() != o.getClass()) return false;

        Label label = (Label) o;
        return (firstPickUpNode == label.firstPickUpNode) && (curNode == label.curNode)
                && (Arrays.equals(Picks.toArray(), label.Picks.toArray()));
    }

    @Override
    public int hashCode() {
        int first = Objects.hash(firstPickUpNode);
        int curr = Objects.hash(curNode);
        int[] orders = Picks.stream().mapToInt(Integer::valueOf).toArray();
        int result = 31 * (first + curr) + Arrays.hashCode(orders);
        return result;
    }

}
