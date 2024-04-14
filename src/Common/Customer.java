package Common;

import java.util.Objects;

public class Customer {
    public int id;  //customer node ID: 0<=id<=2n+1
    public String pick_or_deli; //the physical location of the node
    public double early;  //earliest pickup time
    public double last;  //latest delivery time
    public double serve; //loading/unloading time
    public double demand;  //order size
    public double maxDelay;  //maximum delay time in delivery
    public double lateCost;  //unit lateness cost

    public int route;  //the id of route for serving the order
    public double singleCost;  //LTL cost

    public Customer( ) {
    }

    public Customer(int id, String pick_or_deli, double demand, Double early, Double last, double serve,double singleCost,double maDelay,double lateCost) {
        this.id = id;
        this.pick_or_deli = pick_or_deli;
        this.demand = demand;
        this.early = early;
        this.last = last;
        this.serve = serve;
        this.singleCost = singleCost;
        this.maxDelay = maDelay;
        this.lateCost = lateCost;
    }

    public Customer clonecustomer( ){
        Customer clone = new Customer();
        clone.id = this.id;
        clone.pick_or_deli = this.pick_or_deli;
        clone.early = this.early;
        clone.last = this.last;
        clone.serve = this.serve;
        clone.demand = this.demand;
        clone.singleCost = this.singleCost;
        clone.maxDelay = this.maxDelay;
        clone.lateCost = this.lateCost;
        clone.route = this.route;
        return clone;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Customer customer = (Customer) o;
        return id == customer.id;
    }

    @Override
    public int hashCode() {
        return Objects.hash(id);
    }
}
