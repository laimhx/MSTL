package Common;

import java.util.*;

public class Coalition {
    public HashSet<Integer> member; //orders in the coalition
    public double cost; //cost of coalition

    public Coalition() {
        this.member = new HashSet<>();
        this.cost = Double.MAX_VALUE;
    }

    public HashSet<Integer> getMember() {
        return member;
    }

    public double getCost() {
        return cost;
    }

    public void setCost(long cost) {
        this.cost = cost;
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Coalition coal = (Coalition) o;
        return Objects.equals(member, coal.member);
    }

    @Override
    public int hashCode() {
        return Objects.hash(member);
    }

}
