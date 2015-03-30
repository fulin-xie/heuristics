#include <iostream>
#include <iomanip>
#include <math.h>
#include "dataClass.h"

double VehicleCapacity = 200;

// constructor and functions for the class 'Node'
Node::Node (int Id, double x, double y, double ReadyT, double DueT, double time){
    this->ExternalId = Id;
    this->xcoord = x;
    this->ycoord = y;
    this->ReadyTime0 = ReadyT;
    this->DueDate0 = DueT;
    this->timeAtNode = time;
    this->DriveTimeToNextNode = 0;
}
// default constructor
Node::Node(){
    id = 0;
    ExternalId = 0;
    xcoord = 0;
    ycoord = 0;
    ReadyTime0 = 0;
    DueDate0 = 0;
    timeAtNode = 0;
    this->DriveTimeToNextNode = 0;
}

double Node::Xcoord(){
    return this->xcoord;
}
double Node::Ycoord(){
    return this->ycoord;
}
double Node::TimeToNode(Node NodeTo){
    return sqrt(pow(this->xcoord-NodeTo.xcoord, 2)+ pow(this->ycoord-NodeTo.ycoord, 2));
}
double Node::TimeAtNode(){
    return this->timeAtNode;
}

// constructor for the class 'Depot'
Depot::Depot(int externalId, double x, double y, double ReadyT, double DueT)
            : Node(externalId, x, y, ReadyT, DueT, 0){}

// constructors and functions for the class 'Custormer'
Customer::Customer(int ExternalId, double x, double y, double demand,
       double ReadyTime, double DueDate, double ServiceTime)
        :Node(ExternalId,x,y,ReadyTime, DueDate, ServiceTime)
{
    this->demand0 = demand;
    this->ServiceCompleteTime =0;
}
Customer::Customer() : Node(){
    demand0 = 0;
    ServiceCompleteTime = 0;
    VehicleArrivalTime = 0;
    angle = 0;
    tardiness = 0;
}

double Customer::demand(){
    return this->demand0;
}


//constructors and functions for the class 'Path'
Path::Path(Depot DepotStart, std::vector<Customer>& CustomersVisited)
{
    this->DepotStart = DepotStart;
    this->CustomersVisited = CustomersVisited;
    this->DriveDistance0 =0;
    this->TotalLoad0 = 0;
    this->CapacityViolation0 =0;
    this->DurationViolation0 =0;
    this->TimeWindowViolation0 =0;
    //this->TimeUsed0 = 0;

    if(CustomersVisited.empty()==false){
        GetDriveDistance();
        GetTotalLoad();
        GetTimeInformation();
        GetCapacityViolation();
        GetDurationViolation();
        GetTimeWindowViolation();
    }
}

Path::Path(Depot DepotStart, std::vector<Customer>& CustomersVisited, double DriveDistance, double TotalLoad)
{
    this->DepotStart = DepotStart;
    this->CustomersVisited = CustomersVisited;
    this->DriveDistance0 =DriveDistance;
    this->TotalLoad0 = TotalLoad;
    this->CapacityViolation0 =0;
    this->DurationViolation0 =0;
    this->TimeWindowViolation0 =0;
    //this->TimeUsed0 = 0;

    if(CustomersVisited.empty()==false){
        GetTimeInformation();
        GetCapacityViolation();
        GetDurationViolation();
        GetTimeWindowViolation();
    }
}

void Path::GetDriveDistance(){
    double TimeToNextNode;
    TimeToNextNode = DepotStart.TimeToNode(CustomersVisited[0]);
    this->DriveDistance0 = TimeToNextNode;
    DepotStart.DriveTimeToNextNode = TimeToNextNode;
    for (int i=1; i<(int)CustomersVisited.size(); i++){
        TimeToNextNode = CustomersVisited[i-1].TimeToNode(CustomersVisited[i]);
        this->DriveDistance0 += TimeToNextNode;
        CustomersVisited[i-1].DriveTimeToNextNode = TimeToNextNode; //
    }
    // close vehicle routing problem
    TimeToNextNode = CustomersVisited[CustomersVisited.size()-1].TimeToNode(DepotStart);
    this->DriveDistance0 += TimeToNextNode;
    CustomersVisited[CustomersVisited.size()-1].DriveTimeToNextNode = TimeToNextNode;
}

void Path::GetTotalLoad(){
    for(int i=0; i<(int)CustomersVisited.size();i++){
        this->TotalLoad0 += CustomersVisited[i].demand();
    }
}

void Path::GetTimeInformation(){
    double VehicleArrivalTime = 0;

    VehicleArrivalTime = DepotStart.DriveTimeToNextNode;
    CustomersVisited[0].VehicleArrivalTime = VehicleArrivalTime;
    CustomersVisited[0].ServiceCompleteTime =
                    GetServiceCompleteTime(VehicleArrivalTime,CustomersVisited[0]);

    if(VehicleArrivalTime > CustomersVisited[0].DueDate()){
        CustomersVisited[0].tardiness = VehicleArrivalTime - CustomersVisited[0].DueDate();
    }
    else{
        CustomersVisited[0].tardiness = 0;
    }

    // for all customers after the first customer
    for(int i=1; i<(int)CustomersVisited.size(); i++){
        VehicleArrivalTime = CustomersVisited[i-1].ServiceCompleteTime
                      +CustomersVisited[i-1].DriveTimeToNextNode;
                       // + CustomersVisited[i-1].TimeToNode(CustomersVisited[i]);
        CustomersVisited[i].VehicleArrivalTime = VehicleArrivalTime;
        CustomersVisited[i].ServiceCompleteTime =
                    GetServiceCompleteTime(VehicleArrivalTime, CustomersVisited[i]);
        if(VehicleArrivalTime > CustomersVisited[i].DueDate()){
            CustomersVisited[i].tardiness = VehicleArrivalTime - CustomersVisited[i].DueDate();
        }
        else{
            CustomersVisited[i].tardiness = 0;
        }
    }

    this->TimeUsed0 = CustomersVisited[CustomersVisited.size()-1].ServiceCompleteTime
                                       + CustomersVisited[CustomersVisited.size()-1].TimeToNode(DepotStart);
}

double Path::GetServiceCompleteTime(double VehicleArriveTime, Customer customer){
    double ServiceCompleteTime = 0;
    // if the vehicle arrives before the open of the time window
    if(VehicleArriveTime <= customer.ReadyTime()){
        ServiceCompleteTime = customer.ReadyTime() +customer.TimeAtNode();
    }
    else{
        ServiceCompleteTime = VehicleArriveTime + customer.TimeAtNode();
    }
    return ServiceCompleteTime;
}

void Path::GetCapacityViolation(){
    if((this->TotalLoad0-VehicleCapacity)>0){
        this->CapacityViolation0 = this->TotalLoad0 - VehicleCapacity;
    }
    else{
        CapacityViolation0 =0;
    }
}

void Path::GetDurationViolation(){
    double JobCompleteTime = 0;

    JobCompleteTime = CustomersVisited[CustomersVisited.size()-1].ServiceCompleteTime
                                   + CustomersVisited[CustomersVisited.size()-1].TimeToNode(DepotStart);

    if((JobCompleteTime > DepotStart.DueDate())){
        this->DurationViolation0 = JobCompleteTime-DepotStart.DueDate();
    }
    else{
        this->DurationViolation0 = 0;
    }
}

void Path::GetTimeWindowViolation(){
    double TotalViolation = 0;

    for(int i=0; i<(int)CustomersVisited.size();i++){
        TotalViolation += CustomersVisited[i].tardiness;
    }
    this->TimeWindowViolation0 = TotalViolation;

    /*
    for(int i=0; i<(int)CustomersVisited.size();i++){
        double diff;
        diff = CustomersVisited[i].VehicleArrivalTime - CustomersVisited[i].DueDate();
        if(diff>0){
            TotalViolation += diff;
        }
    }
    */
    this->TimeWindowViolation0 = TotalViolation;
}

void Path::GetObjectiveValue(double alpha, double beta, double gamma){
    double objective = 0;
    objective = this->DriveDistance0 + alpha*CapacityViolation0 + beta*DurationViolation0 + gamma*TimeWindowViolation0;

    this->ObjectiveValue0 = objective;
}

void Path::DisplayInfo() {
    std::cout << std::endl << "Path id: " << id0 << std::endl;
    std::cout << "Stops : depot->";
    // the output id is the external id which is the id from the external file
    for (std::vector<Customer>::iterator it = CustomersVisited.begin();
         it != CustomersVisited.end(); it ++)
    {
        Customer c = *it;
        std::cout << (c.ExternalId - 1) << " -> "; // make the depot has id 0, customer Id start from 1
    }
    std::cout << std::endl << "DriveDistance: " << std::setprecision(6)<< DriveDistance0 // << "  Time Used: " << TimeUsed0
              << " Capacity Violation: " << this->CapacityViolation()
              << " Duration Violation: " << this->DurationViolation()
              << " Time Window Violation: " << this->TimeWindowViolation() <<std::endl;
}

Solution::Solution(std::vector<Path*> Paths, bool IsNeighborSolution)
{
    this->PathList0 = Paths;
    this->NumOfPaths0 = PathList0.size();
    // if this solution is not neighbor solution, then calculate solution values
    if(IsNeighborSolution==false){
        this->TotalDriveDistance0 = 0;
        this->TotalCapacityViolation0 = 0;
        this->TotalDurationViolation0 = 0;
        this->TotalTimeWindowViolation0 = 0;
        if(PathList0.empty()==false){
            GetTotalDistanceAndTime();
            GetViolations();
        }
    }
}

void Solution::GetTotalDistanceAndTime()
{
    double DriveDistance = 0;


    if(PathList0.empty()==false){
        for(int i=0; i<(int)PathList0.size();i++){
            DriveDistance += PathList0[i]->DriveDistance();
        }
    }
    this->TotalDriveDistance0 = DriveDistance;

}

void Solution::GetViolations()
{
    double CapacityViolation = 0;
    double DurationViolation = 0;
    double TimeWindowViolation = 0;

    if(PathList0.empty()==false){
        for(int i=0; i<(int)PathList0.size();i++){
             CapacityViolation += PathList0[i]->CapacityViolation();
             DurationViolation += PathList0[i]->DurationViolation();
             TimeWindowViolation += PathList0[i]->TimeWindowViolation();
        }
    }
    this->TotalCapacityViolation0 = CapacityViolation;
    this->TotalDurationViolation0 = DurationViolation;
    this->TotalTimeWindowViolation0 = TimeWindowViolation;
}

void Solution::GetObjectiveValue(double alpha, double beta, double gamma){
    double objective = 0;
    objective = TotalDriveDistance0 + alpha*TotalCapacityViolation0 + beta*TotalDurationViolation0 + gamma*TotalTimeWindowViolation0;

    //objective = alpha*TotalCapacityViolation0 + beta*TotalDurationViolation0 + gamma*TotalTimeWindowViolation0;
    this->ObjectiveValue0 = objective;
}

double Solution::TotalViolation()
{
    double violations = 0;
    violations = TotalCapacityViolation0 + TotalDurationViolation0 + TotalTimeWindowViolation0;
    return violations;
}

void Solution::DisplaySolution()
{
    int TotalCustomer = 0;

    for(int i=0; i<(int)PathList0.size(); i++){
        PathList0[i]->DisplayInfo();
        std::vector<Customer> CustomersList  = PathList0[i]->CustomersList();
        TotalCustomer += CustomersList.size();
    }

    std::cout << std::endl << "Total Drive Distance: " << std::setprecision(6) << TotalDriveDistance0 << std::endl
         // << "Total Time Used: " << TotalTimeUsed0 << std::endl
            << "Total Capacity Violation: " << TotalCapacityViolation0 << std::endl
               << "Total Duration Violation: " << TotalDurationViolation0 << std::endl
               << "Total Time window Violation: " << TotalTimeWindowViolation0 << std::endl;

    //std::cout << std::endl << TotalCustomer << std::endl;
}


