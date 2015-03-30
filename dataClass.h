#ifndef DATACLASS_H
#define DATACLASS_H
#include <vector>
#include <map>

extern double VehicleCapacity;

// the class of all notes
class Node
 {
    double xcoord;
    double ycoord;
    double ReadyTime0;
    double DueDate0;
    double timeAtNode; //the time spent at the node, usually the service time
public:
    int id; // this is the node id in the NodeList, Depot id is -1, customer id start from 0
    int ExternalId; // the id from the raw data source
    Node (int Id, double x, double y, double ReadyT, double DueT, double time);
    double Xcoord();
    double Ycoord();
    double ReadyTime() {return this->ReadyTime0;}
    double DueDate() {return this->DueDate0;}
    double TimeToNode(Node NodeTo);
    double TimeAtNode();
    double DriveTimeToNextNode; // in a route containt a series of nodes, the drive time from current node to next node
    //default constructor
    Node();
 };

// the class of the depot
class Depot : public Node
{
public:
    Depot(int externalId, double x, double y, double ReadyT, double DueT);
    Depot() : Node(){}
};

// the class of the Customer
class Customer : public Node
{
private:
    double demand0;
public:
    Customer(int ExternalId, double x, double y, double demand,
           double ReadyTime, double DueDate, double ServiceTime);
    double angle;
    double demand();
    double tardiness;
    double ServiceCompleteTime;
    double VehicleArrivalTime;
    Customer();
};

// the class of the Path
class Path
{
private:
    double CapacityViolation0, DurationViolation0, TimeWindowViolation0;
    double TimeUsed0; // the time finshed all jobs in the path, the total time used in the path
    double DriveDistance0;
    double TotalLoad0;
    double ObjectiveValue0;
    std::vector<Customer> CustomersVisited;
    bool CalculateAllData0;

    // calculate the total cost of this path
    void GetDriveDistance();
    void GetTotalLoad();
    void GetCapacityViolation();
    void GetDurationViolation();
    void GetTimeWindowViolation();
    void GetTimeInformation(); // calculate the vehicle arrive time at each customer, service complete time of each customer
public:
    int id0;
    Depot DepotStart;
    // list of customers visited in this path
    Path(Depot DepotStart, std::vector<Customer>& CustomersVisited);
    Path(Depot DepotStart, std::vector<Customer>& CustomersVisited, double DriveDistance, double TotalLoad);
    Path(): CapacityViolation0(0), DurationViolation0(0), TimeWindowViolation0(0), DriveDistance0(0), TotalLoad0(0),
                                                                CustomersVisited(), id0(0){} // default constructor
    void DisplayInfo();
    void SetId(int id){this->id0 = id;}
    double CapacityViolation(){return this->CapacityViolation0;}
    double DurationViolation(){return this->DurationViolation0;}
    double TimeWindowViolation(){return this->TimeWindowViolation0;}
    double TimeUsed(){return this->TimeUsed0;}
    double GetServiceCompleteTime(double VehicleArriveTime, Customer customer);
    std::vector<Customer> CustomersList(){return this->CustomersVisited;}
    double ObjectiveValue(){return this->ObjectiveValue0;}
    void GetObjectiveValue(double alpha, double beta, double gamma);
    double DriveDistance(){return this->DriveDistance0;}
    double TotalLoad(){return this->TotalLoad0;}
    int id(){return this->id0;}
};


// the class of Solution
class Solution{
private:
    int NumOfPaths0;
    double ObjectiveValue0;
    std::vector<Path*> PathList0; // the list of path in the current list
    void GetTotalDistanceAndTime();
    void GetViolations();

protected:
    double TotalDriveDistance0;
    double TotalCapacityViolation0, TotalDurationViolation0, TotalTimeWindowViolation0;

public:
    //int id; //the id of the CurrentSolution
    int NeighborId; // the id of the solution in the neighbor lists
    Solution(std::vector<Path*> Paths, bool IsNeighborSolution);

    Solution(): NumOfPaths0(0), TotalDriveDistance0(0), TotalCapacityViolation0(0),
         TotalDurationViolation0(0), TotalTimeWindowViolation0(0), PathList0(){} // default constructor

    void DisplaySolution();
    std::vector<Path*> PathList(){return this->PathList0;}
    void SetPathList(std::vector<Path*> PathPointerList) {this->PathList0 = PathPointerList;}
    double TotalDriveDistance(){return this->TotalDriveDistance0;}

    int NumOfPaths(){return this->NumOfPaths0;}
    double ObjectiveValue(){return this->ObjectiveValue0;}
    double TotalCapacityViolation(){return this->TotalCapacityViolation0;}
    double TotalDurationViolation(){return this->TotalDurationViolation0;}
    double TotalTimeWindowViolation(){return this->TotalTimeWindowViolation0;}
    void GetObjectiveValue(double alpha, double beta, double gamma);
    double TotalViolation();
};

//the class of the neighbor solution
class NeighborSolution : public Solution
{
    //index of the moves, which shows how neighbor Solution is obtained from the current solution
    int PathOneId0;
    int CustomerOneId0;
    int PathTwoId0;
    int CustomerTwoId0;
    bool FirstImprovedNeighbor0;

public:
    NeighborSolution(std::vector<Path*> Paths, double TotalDistance, double CapacityViolation,
                     double DurationViolation, double TimeWindowViolation, int PathOneId,
                     int CustomerOneId,int PathTwoId, int CustomerTwoId, bool IsNeighborSolution)
                     : Solution(Paths, IsNeighborSolution){

        this->TotalDriveDistance0 = TotalDistance;
        this->TotalCapacityViolation0 = CapacityViolation;
        this->TotalDurationViolation0 = DurationViolation;
        this->TotalTimeWindowViolation0 = TimeWindowViolation;
        this->PathOneId0 = PathOneId;
        this->CustomerOneId0 = CustomerOneId;
        this->PathTwoId0 = PathTwoId;
        this->CustomerTwoId0 = CustomerTwoId;
    }

    NeighborSolution(std::vector<Path*> Paths, int PathOneId, int CustomerOneId,
                     int PathTwoId, int CustomerTwoId, bool IsNeighborSolution)
                     : Solution(Paths, IsNeighborSolution){

        this->PathOneId0 = PathOneId;
        this->CustomerOneId0 = CustomerOneId;
        this->PathTwoId0 = PathTwoId;
        this->CustomerTwoId0 = CustomerTwoId;
    }

    NeighborSolution(): Solution(), PathOneId0(0), CustomerOneId0(0),PathTwoId0(0), CustomerTwoId0(0), FirstImprovedNeighbor0(false){}

    int PathOneId(){return this->PathOneId0;}
    int CustomerOneId() {return this->CustomerOneId0;}
    int PathTwoId(){return this->PathTwoId0;}
    int CustomerTwoId(){return this->CustomerTwoId0;}
    void SetFirstImprovedNeighbor (bool FirstImprovedNeighbor) {this->FirstImprovedNeighbor0 = FirstImprovedNeighbor;}
    bool FirstImprovedNeighbor() {return this->FirstImprovedNeighbor0;}

};


#endif // DATA_H
