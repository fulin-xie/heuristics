#include <iostream>
#include <map>
#include <math.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <time.h>

#include "tabusearch.h"
#include "dataClass.h"
#include "localSearch.h"


using namespace std;

double TabuSearch::epsilon = 0.00001;

TabuSearch::TabuSearch()
{
    //this->FilePath = "/Users/fulin/Documents/PhD/C++/OVRPTW/Instances2.txt";
    //this->FilePath = "/Users/fulin/Documents/PhD/C++/TabuSearch/heuristics//InstancesR101.txt";
    this->FilePath = "C:/C++/VRPTW//heuristics/InstancesR102.txt";
    //configurations
    this->VehicleCount = 17;
    this->MaxIterNum = 50000; // maximum number of iterations
    this->MaxConsIterNum = 500000; // maximum number of consecutive iterations without improvement to the best solution
    this->TabuMemoryLength = 15;
    this->delta = 0.1; // factor used to modify the three parameters above
    this->lambda = 0.015; //factor used to control the intensity of the diversification

    //other variables
    this->alpha = 1;
    this->beta = 1;
    this->gamma = 1;
    this->LocalSearchRule = 1;
    this->NumOfNeighbors = 200;
    this->FeasibleSolutionCost = 10000000000; // initialize with a huge value
    this->BestObjectiveValue = 100000000000; // the best normalise solution value found so far. it is initilise with a big value
    this->FeasibleSolutionFound = false;
    this->IterNum = 0; //Current number of iterations
    this->ComputationTimeInMinute = 0;
    this->CurrentFrequency = 0;
    this->ConsIterNum = 0; //current number of consecutive iterations without any improvement
    this->ScalingFactor = 0;
}

void TabuSearch::RunModel()
{
    DataInitialization(this->FilePath);
    //VerifyDistanceCal();
    double MinuteInSecond = 60;
    int t0 = (int)time(NULL);
    SortCustomers();
    //DisplayCustomerData();
    GetInitialSolution();
    RunTabuSearch(MaxIterNum);
    int t1 = (int)time(NULL);
    double ComputationTime = t1 -t0; // the computation time in seconds
    ComputationTimeInMinute = ComputationTime / MinuteInSecond;
    ComputationTimeInSecond = ComputationTime;
    DisplaySolution();
    cout << "Random Number: " << RandomNum << endl;
    CleanUp(); // clean the memory

}

void TabuSearch::DisplaySolution()
{
    if(FeasibleSolutionFound == true){
        FeasibleSolution.DisplaySolution();
    }
    else{
        cout << "No Feasible solution found";
        SolutionList[SolutionList.size()-1].DisplaySolution();
    }

    cout << "Best normalized objective value: " << BestObjectiveValue << endl;
    //cout << "Computation time in minutes: " << ComputationTimeInMinute << endl;
    cout << "Computation time in second: " << ComputationTimeInSecond << endl;
    /*
    int count = 0;
    for(int i=0; i<CustomerCount; i++)
        for(int j=0; j<VehicleCount; j++)
        {
            count ++;
            cout << " Visited Times:  " << attribute[i][j].VisitedTimes() << endl;
        }
    cout << count << endl; */
}


void TabuSearch::DisplayNeighborSolution(std::vector<NeighborSolution> &NeighborSolutions)
{
    int size = NeighborSolutions.size();

    SolutionList[SolutionList.size()-1].DisplaySolution();

    for(int i=0; i<NeighborSolutions.size(); i++){
        NeighborSolution solution =  NeighborSolutions[i];
        cout << solution.CustomerOneId() << " && " << solution.PathOneId() << " && " << solution.PathTwoId()
             << "  Objective value:  " << solution.ObjectiveValue() << endl;

        /*
        vector<Path*> pathlist;
        pathlist = solution.PathList();
        Path NewPathIn = * pathlist[pathlist.size()-2];
        NewPathIn.DisplayInfo();
        */
      //  solution.DisplaySolution();
    }
}

bool TabuSearch::CheckFeasibility(Solution& solution)
{
    bool feasible = false;

    if(solution.TotalViolation()<=0){
        feasible = true;
    }
    return feasible;
}

void TabuSearch::RunTabuSearch(int MaxIterNum)
{
    this->ScalingFactor = sqrt(CustomerCount*VehicleCount);

    Solution CurrentSolution = SolutionList[0];
    BestSolution = CurrentSolution;
    int IterNumNoImprove = 0;

    while((IterNum < MaxIterNum) && (ConsIterNum < MaxConsIterNum)){
        //generate all neighbor solutions
        IterNum = IterNum + 1;
        /* randomly select a tabu length
        srand(time(0));
        TabuMemoryLength = rand() % 30; //generate a random number in the rand 0 and 99, it determines the first selected cust
        */
        /*
        if(IterNumNoImprove >= 100){
            ++ LocalSearchRule; // use another local search rule
            IterNumNoImprove = 0; //reset the count to 0
            if(LocalSearchRule > 3){ //reset to the rule one
                LocalSearchRule = 1;
            }
        }
        */
        /*
        if((IterNumNoImprove >= 100) && LocalSearchRule==3){
            LocalSearchRule = 1;
            ResetAttribute();
            IterNumNoImprove = 0;
        }
        if(IterNumNoImprove >= 100){
            ++ LocalSearchRule;
            IterNumNoImprove = 0;
            //CurrentSolution = BestSolution;
        }
        */
        GetAllNeighborSolutions(CurrentSolution); // call the local search function to get neighborhood solutions
        NeighborSolution BestNeighborSolution = BestNeighbor(LocalSearch::AllNeighbors, CurrentSolution);
       // DisplayNeighborSolution(LocalSearch::AllNeighbors);
        CurrentSolution = BestNeighborSolution;

        double NormalisedObjValue = BestNeighborSolution.TotalViolation()
                                                       + BestNeighborSolution.TotalDriveDistance();
        /*
        if((BestObjectiveValue-NormalisedObjValue) > epsilon ){
            BestObjectiveValue = NormalisedObjValue;
            ConsIterNum = 0;
            IterNumNoImprove = 0;
        } */
        if((BestSolution.TotalDriveDistance() - BestNeighborSolution.TotalDriveDistance() > epsilon)
                && (BestSolution.TotalViolation() - BestNeighborSolution.TotalViolation() > epsilon)){
            ConsIterNum = 0;
            IterNumNoImprove = 0;
            BestSolution = BestNeighborSolution;
        }
        else{
            ConsIterNum = ConsIterNum + 1;
            IterNumNoImprove = IterNumNoImprove + 1;
        }

        if(CheckFeasibility(CurrentSolution)==true &&
                CurrentSolution.TotalDriveDistance() < FeasibleSolutionCost){
            FeasibleSolution = CurrentSolution;
            FeasibleSolutionCost = FeasibleSolution.TotalDriveDistance();
            FeasibleSolutionFound = true;
        }
        UpdateParameters(CurrentSolution);
        SolutionList.push_back(CurrentSolution);

        //pause the loop, until the user clicks the enter
        if((IterNum % 10000) == 0){
            if(FeasibleSolutionFound==true){FeasibleSolution.DisplaySolution();}
            cout << "Best normalized objective value: " << BestObjectiveValue << endl;
            cout << "Random Number" << RandomNum << endl;
            if(cin.get() == '\n'){}
        }
    }
}

void TabuSearch::GetAllNeighborSolutions(Solution& CurrentSolution)
{
    vector<NeighborSolution> AllNeighborSolutions; // the list of neighbor solutions
    LocalSearch search;
    if(LocalSearchRule == 1){
        search.relocate(CurrentSolution, *DepotStart, BestObjectiveValue,alpha, beta, gamma, attribute);
    }
    else if(LocalSearchRule == 2){
        search.swap(CurrentSolution, *DepotStart,alpha, beta, gamma);
    }
    else{   
         search.TwoOptAsterisk(CurrentSolution, *DepotStart,alpha, beta, gamma);
    }
}


double TabuSearch::GetPenalty(double cost, int MoveFrequency)
{
    double Penalty = 0;

    Penalty = lambda*cost*ScalingFactor*MoveFrequency;
    //int num = IterNum;
    //Penalty = 0;
    return Penalty;
}

// get the best non tabu solution
NeighborSolution TabuSearch::BestNeighbor(vector<NeighborSolution>& NeighborSolutions, Solution CurrentSolution)
{
    NeighborSolution BestNeighborSolution;
    int PathOneId, CustomerOneId, PathTwoId, CustomerTwoId;
    bool BestNonTabuNeighborFound = false;
    double Penalty = 0;

    while(BestNonTabuNeighborFound == false){
        int BestSolutionPosition = 0;
        double MinObjValue = 1000000000; // initialise with a huge value
        int NeighborsCount = NeighborSolutions.size();
        for(int i=0; i<NeighborsCount; i++){
            if((MinObjValue-NeighborSolutions[i].ObjectiveValue())>epsilon){
                BestNeighborSolution = NeighborSolutions[i];
                MinObjValue = BestNeighborSolution.ObjectiveValue();
                BestSolutionPosition = i;
            }
        }

        if(LocalSearchRule == 3){
            goto TailExchange;
        }

        //diversification process; if the objective value of best neighbor solution is
        // higher than the current solution, then check the penalty, the objectice function is minimize
        if((BestNeighborSolution.ObjectiveValue()-CurrentSolution.ObjectiveValue()) > epsilon){
            Diversification(NeighborSolutions, BestNeighborSolution, BestSolutionPosition);
        }

        TailExchange:
        //get the move indicators from the best neighbor solution
        CustomerOneId = BestNeighborSolution.CustomerOneId();
        PathOneId = BestNeighborSolution.PathOneId();
        PathTwoId = BestNeighborSolution.PathTwoId();
        CustomerTwoId = BestNeighborSolution.CustomerTwoId();

        // check the best neighbot solution is tabu or statisfied aspiration cretiarion
        if(AcceptNeighbor(BestNeighborSolution) == false){
            NeighborSolutions.erase(NeighborSolutions.begin()+BestSolutionPosition);
            BestNonTabuNeighborFound = false;
        }
        else{BestNonTabuNeighborFound = true;}
    }
    {
        //update the tabu list, and the frequency map
        UpdateTabuList(BestNeighborSolution);

        if(LocalSearchRule==1 || LocalSearchRule==2){
            //call function to update the current frequency
            CurrentFrequency = GetFrequency(BestNeighborSolution, true);

            cout << CustomerOneId << " && " << PathOneId << " && " << PathTwoId <<
                            "  Iteration: " << IterNum << "  Frequency:  " << CurrentFrequency
                << "  Objective value:  " << BestNeighborSolution.ObjectiveValue() << " Penalty : " << Penalty << endl;
        }
        if(LocalSearchRule == 3){
            cout << "TailExchange" << " Iteration: " << IterNum <<
                 " CustomerOneId: "<< BestNeighborSolution.CustomerOneId()<<
                 " CustomerTwoId: " << BestNeighborSolution.CustomerTwoId() <<endl;
        }
    }

    Path PathOne = *BestNeighborSolution.PathList()[BestNeighborSolution.NumOfPaths()-1];
    Path PathTwo = *BestNeighborSolution.PathList()[BestNeighborSolution.NumOfPaths()-2];
    //insert the two new created paths into the AllPathList
    LocalSearch::AllPathList.push_back(PathOne);
    LocalSearch::AllPathList.push_back(PathTwo);
    list<Path>::iterator it = LocalSearch::AllPathList.end();
    // Note PathList is a private member, can not access outside of the class
    vector<Path*> PathPointerList = BestNeighborSolution.PathList();
    PathPointerList.pop_back();
    PathPointerList.pop_back();

    --it;
    PathPointerList.push_back(& *it);
    --it;
    PathPointerList.push_back(& *it);
    BestNeighborSolution.SetPathList(PathPointerList);

    return BestNeighborSolution;
}

//if find the neighbor solution with the minimal (objective fucntion plus penalty)
void TabuSearch::Diversification(std::vector<NeighborSolution> &NeighborSolutions,
                                 NeighborSolution &BestNeighborSolution,
                                 int &BestSolutionPosition){
    // loop again, find the one with minimal objective function plus penalty
    BestSolutionPosition = 0;
    double MinObjValue = 1000000000; // initialise with a huge value
    for(int i=0; i<(int)NeighborSolutions.size(); i++){
        double TotalDriveDistance = NeighborSolutions[i].TotalDriveDistance();
        int CustomerOneId = NeighborSolutions[i].CustomerOneId();
        int PathTwoId = NeighborSolutions[i].PathTwoId();

        //int NeighborFrequency = GetFrequency(NeighborSolutions[i], false)-1;
        int NeighborFrequency = attribute[CustomerOneId][PathTwoId].VisitedTimes();

        double TotalObjValue = NeighborSolutions[i].ObjectiveValue()
            + GetPenalty(TotalDriveDistance, NeighborFrequency);

        if((MinObjValue-TotalObjValue)>epsilon){
            BestNeighborSolution = NeighborSolutions[i];
            //MinObjValue = BestNeighborSolution.ObjectiveValue();
            MinObjValue=TotalObjValue;
            BestSolutionPosition = i;
        }
    }
}

bool TabuSearch::AcceptNeighbor(NeighborSolution& BestNeighborSolution)
{
    bool accept = false;

    double cost = BestNeighborSolution.TotalDriveDistance();
    int CustomerOneId = BestNeighborSolution.CustomerOneId();
    int CustomerOneDescId = BestNeighborSolution.CustomerOneDescId();
    int PathOneId = BestNeighborSolution.PathOneId();
    int PathTwoId = BestNeighborSolution.PathTwoId();
    int CustomerTwoId = BestNeighborSolution.CustomerTwoId();
    int CustomerTwoDescId = BestNeighborSolution.CustomerTwoDescId();

    // if the new neighbor has a small objective value than the best objective valuse found so far
    // the relocate operator
    switch (LocalSearchRule){
    case 1:// the relocate operator
        if(attribute[CustomerOneId][PathTwoId].AspirationLevel() - cost > epsilon){ //aspiratioin criteria
            attribute[CustomerOneId][PathTwoId].SetAspirationLevel(cost); // set the aspiration level
            accept = true;
        }
        else{ // check if it is tabu
            if(attribute[CustomerOneId][PathTwoId].TabuStatus() > 0){accept = false;} // a tabu move
            else{accept = true;}
        }
        break;
    case 2: // the swap operator
        if((attribute[CustomerOneId][PathTwoId].AspirationLevel() - cost > epsilon) &&
                (attribute[CustomerTwoId][PathOneId].AspirationLevel() - cost > epsilon)){ //aspiratioin criteria
            attribute[CustomerOneId][PathTwoId].SetAspirationLevel(cost); // set the aspiration level
            attribute[CustomerTwoId][PathOneId].SetAspirationLevel(cost);
            accept = true;
        }
        else{ // check if it is tabu, if one of them is tabu, then is a tabu neighborhood solution
            if((attribute[CustomerOneId][PathTwoId].TabuStatus()>0) || //check the tabu status
                    (attribute[CustomerTwoId][PathOneId].TabuStatus()>0)) {accept = false;}
            else{accept = true;}
        }
        break;
    case 3: //the 2opt operator
        if((ArcAttribute[CustomerOneId][CustomerOneDescId].TabuStatus()>0) ||
                (ArcAttribute[CustomerTwoId][CustomerTwoDescId].TabuStatus()>0)) {accept = false;}
        else {accept = true;}
        break;
    default:
        cout << "no valid neighborhood operator found.";
    }

    return accept;
}

int TabuSearch::GetFrequency(NeighborSolution& BestNeighborSolution, bool UpdateAttributeFrequency)
{
    int frequency = 0;
    int CustomerOneId = BestNeighborSolution.CustomerOneId();
    int PathOneId = BestNeighborSolution.PathOneId();
    int PathTwoId = BestNeighborSolution.PathTwoId();
    int CustomerTwoId = BestNeighborSolution.CustomerTwoId();

    //update the attribute move number, the frequency map
    frequency = CurrentFrequency - attribute[CustomerOneId][PathOneId].VisitedTimes()
                                        + attribute[CustomerOneId][PathTwoId].VisitedTimes()+1;
    if(CustomerTwoId != -1){
        frequency = frequency - attribute[CustomerTwoId][PathTwoId].VisitedTimes()
                                            + attribute[CustomerTwoId][PathOneId].VisitedTimes()+1;
    }
    // update attributes visited times
    if(UpdateAttributeFrequency == true){
        int TotalVisitedNumber = attribute[CustomerOneId][PathTwoId].VisitedTimes()+1;
        attribute[CustomerOneId][PathTwoId].SetVisitedTimes(TotalVisitedNumber);
        if(CustomerTwoId != -1){
            TotalVisitedNumber = attribute[CustomerTwoId][PathOneId].VisitedTimes()+1;
            attribute[CustomerTwoId][PathOneId].SetVisitedTimes(TotalVisitedNumber);
        }
    }
    return frequency;
}

void TabuSearch::UpdateTabuList(NeighborSolution& BestNeighborSolution)
{
    int CustomerOneId = BestNeighborSolution.CustomerOneId();
    int CustomerOneDescId = BestNeighborSolution.CustomerOneDescId();
    int PathOneId = BestNeighborSolution.PathOneId();
    int PathTwoId = BestNeighborSolution.PathTwoId();
    int CustomerTwoId = BestNeighborSolution.CustomerTwoId();
    int CustomerTwoDescId = BestNeighborSolution.CustomerTwoDescId();

    //update the tabu list
    for(int i=0; i<CustomerCount; ++i){
        for(int j=0; j<VehicleCount; ++j){
            // the removed customer cannot be reinserted into the path in 15 steps
            if(attribute[i][j].TabuStatus() > 0){
                attribute[i][j].SetTabuStatus(attribute[i][j].TabuStatus()-1);
            }
        }
    }
    attribute[CustomerOneId][PathOneId].SetTabuStatus(TabuMemoryLength);
    if(CustomerTwoId != -1){
        attribute[CustomerTwoId][PathTwoId].SetTabuStatus(TabuMemoryLength);
    }

    //the 2-opt operator
    if(LocalSearchRule == 3){
        for(int i=0; i<CustomerCount; ++i){
            for(int j=0; j<CustomerCount; ++j){
                if(ArcAttribute[i][j].TabuStatus()>0){
                    ArcAttribute[i][j].SetTabuStatus(ArcAttribute[i][j].TabuStatus()-1);
                }
            }
        }
        ArcAttribute[CustomerOneId][CustomerOneDescId].SetTabuStatus(TabuMemoryLength);
        ArcAttribute[CustomerTwoId][CustomerTwoDescId].SetTabuStatus(TabuMemoryLength);
    }
}

void TabuSearch::ResetAttribute()
{
    for(int i=0; i<CustomerCount; ++i){
        for(int j=0; j<VehicleCount; ++j){
            attribute[i][j].SetTabuStatus(0);
        }
    }
}

void TabuSearch::ResetArcAttribute()
{
    for(int i=0; i<CustomerCount; ++i){
        for(int j=0; j<CustomerCount; ++j){
            ArcAttribute[i][j].SetTabuStatus(0);
        }
    }
}

void TabuSearch :: UpdateParameters(Solution& CurrentSolution)
{
    if(CurrentSolution.TotalCapacityViolation() > epsilon){
        if(this->alpha < 2000){
            this->alpha = this->alpha * (1+delta); // set an upper bound
        }
    }
    else {
        if(this->alpha > 0.00001){
            this->alpha = this->alpha / (1+delta); // lower bound
        }
    }

    double TotalDurationViolation = CurrentSolution.TotalDurationViolation();
    if(CurrentSolution.TotalDurationViolation() > epsilon){
        if(this->beta < 2000){
            this->beta = this->beta * (1+delta);
        }
    }
    else{
        if(this->beta > 0.00001){
        this->beta = this->beta / (1+delta);
        }
    }

    double TimewindowViolation = CurrentSolution.TotalTimeWindowViolation();

    if(CurrentSolution.TotalTimeWindowViolation() > epsilon){
        if(this->gamma < 2000){
            this->gamma = this->gamma * (1+delta);
        }
    }
    else{
        if(this->gamma > 0.00001){
            this->gamma = this->gamma / (1+delta);
        }
    }
}

void TabuSearch::GetInitialSolution()
{
    srand(time(0));
    //int i = rand() % 100; //generate a random number in the rand 0 and 99, it determines the first selected customer
    int i = 0;
    RandomNum = i;
    int NumOfInsertion = 0; // the number of customers have been inserted to vehicles' route
    int PathCount = 0;
    vector<Path*> PathsInNewSolution;
    while(NumOfInsertion<(int)CustomersSorted.size()){
        vector<Customer> CustomersInPath; // list of customers
        double PathLoad = 0;
        bool NewPathComplete = false;
        while((PathLoad <= VehicleCapacity) && (NumOfInsertion<(int)CustomersSorted.size())&& NewPathComplete == false){
            if(i==100){ //if the end of the list of sorted customers are met, set the i = 0, which is the start of the list
                i=0;
            }
            Customer c= CustomersSorted[i]; // select a customer from the list
            NewPathComplete = true;
            Customer TempCustomer = c;
            vector<Customer> TempCustomersInPath = CustomersInPath;
            if((c.demand()+PathLoad)<VehicleCapacity){
                if(CustomersInPath.empty()){
                    if(DurationConstraint(TempCustomer,0,TempCustomersInPath)==true){
                        // push the copy of the customer into the vector
                        c = TempCustomer;
                        CustomersInPath = TempCustomersInPath;
                        PathLoad = PathLoad + c.demand();
                        i++;
                        NumOfInsertion++;
                        NewPathComplete = false;
                    }
                }
                else{
                    // find the first customer whose ready time is
                    // higher than the inserted customer
                    vector<Customer>::iterator it = CustomersInPath.begin();
                    int index = 0;
                    while(it !=CustomersInPath.end() &&
                          (c.ReadyTime() >= (*it).ReadyTime())){ // if no such insertion place found, the insert customer to the end of the path
                        it++;
                        index++;
                    }
                    if(DurationConstraint(TempCustomer, index, TempCustomersInPath) ==true){
                        c = TempCustomer;
                        CustomersInPath = TempCustomersInPath;
                        PathLoad = PathLoad + c.demand();
                        i++;
                        NumOfInsertion++;
                        NewPathComplete = false;
                    }
                }
                if(NumOfInsertion == (int)CustomersSorted.size()){ //the customer just inserted is the last customer
                    NewPathComplete = true;
                }
            }
            if(NewPathComplete == true){ //the path capacity is full

                //if this path is the last vehicle, and there are customers left, insert customer into this path
                if(((int)PathsInNewSolution.size() == (VehicleCount-1)) && (NumOfInsertion<(int)CustomersSorted.size())){
                    while(NumOfInsertion<(int)CustomersSorted.size()){
                        Customer CustomerLeft = CustomersSorted[i];
                        vector<Customer>::iterator it = CustomersInPath.begin();
                        while(it !=CustomersInPath.end() &&
                              (c.ReadyTime() >= (*it).ReadyTime())){
                            it++;
                        }
                        CustomersInPath.insert(it, CustomerLeft);
                        i++;
                        NumOfInsertion++;
                    }
                }
                Path NewPath(*DepotStart, CustomersInPath);
                NewPath.SetId(PathCount);
                LocalSearch::AllPathList.push_back(NewPath);
                list<Path>::iterator it1 = LocalSearch::AllPathList.end();
                --it1; // iterator to the last item of the list
                PathsInNewSolution.push_back(& *it1); //store the address of the last item into the path list of the new solution

                PathCount++;
            }
        }
    }
    while((int)PathsInNewSolution.size() < VehicleCount){ //insert other empty routes into the solution
        Path NewPath;
        NewPath.SetId(PathCount);
        LocalSearch::AllPathList.push_back(NewPath);
        list<Path>::iterator it2 = LocalSearch::AllPathList.end();
        --it2; // iterator to the last item of the list
        PathsInNewSolution.push_back(& *it2);
        PathCount++;
    }
    cout <<" Path Count:  " << PathsInNewSolution.size() << endl;
    Solution NewSolution(PathsInNewSolution, false);
    NewSolution.GetObjectiveValue(alpha, beta, gamma);
    SolutionList.push_back(NewSolution); // store the solution into the solution list
    GetInitialAspirationLevel(NewSolution);
}

void TabuSearch::GetInitialAspirationLevel(Solution& solution)
{
    vector<Path* > PathList = solution.PathList();
    double cost = solution.TotalDriveDistance();
    for(int i=0; i<(int)PathList.size(); ++i){
        Path path = *PathList[i];
        vector<Customer> CustomerList = path.CustomersList();
        for(int j=0; j<(int)CustomerList.size(); ++j){
            attribute[j][i].SetAspirationLevel(cost);
            // set the initial visited times
            attribute[j][i].SetVisitedTimes(attribute[j][i].VisitedTimes()+1);
        }
    }
    PathList.clear();
}

bool TabuSearch::DurationConstraint(Customer &InsertCustomer, int InsertPosition,
                                    std::vector<Customer> &CustomersVisited){

    vector<Customer>::iterator it = CustomersVisited.begin();
    advance(it, InsertPosition);
    // insert customer into the path, at least one customer in the path
    CustomersVisited.insert(it,InsertCustomer);
    Path NewPath(*DepotStart, CustomersVisited);
    if(NewPath.DurationViolation() <= 0){
        return true; // statisfied the duration constraint
    }
    else{
        return false;
    }
}

void TabuSearch::SortCustomers()
{
    const double PI=3.14159265;
    //sort the customer in by the increasing order of the anlgle they
    // made with depot and an arbitrary radius
    // calculate the angles
    for(map<int, Customer*>::iterator it=CustomerMap.begin();
            it!=CustomerMap.end(); it++){
        double x, y, angle;
        Customer* c = it->second;
        x = c->Xcoord() - DepotStart->Xcoord();
        y = c->Ycoord() - DepotStart->Ycoord();
        angle = atan2(y,x)*180/PI;
        if(angle >= 0){
            c->angle = angle;
        }
        else{
            c->angle = 360 + angle;
        }
        CustomersSorted.push_back(*c);
    }
    std::sort(CustomersSorted.begin(), CustomersSorted.end(),DataSortPredicate);
}

bool TabuSearch::DataSortPredicate(const Customer& c1, const Customer& c2){
    bool c1IsFront = false;
    if(c2.angle - c1.angle > epsilon){
        c1IsFront = true;
    }
    else if(c2.angle == c1.angle){ // these two customers have the same angle
        if(c1.id < c2.id){
            c1IsFront = true;
        }
    }
    else{
        c1IsFront = false;
    }

    return c1IsFront;
}

void TabuSearch::DisplayCustomerData()
{   /*
    for(map<int,Customer*>::iterator it=CustomerMap.begin();
            it!=CustomerMap.end(); it++){
        Customer *c = it->second;
        cout<< c->id << "  " << c->angle << endl;
    }*/

    for(int i=0; i<(int)CustomersSorted.size(); i++){ //the customer Id start from 1
        cout << "ID:  "<< (CustomersSorted[i].ExternalId - 1) << "   Angle:  " << setprecision(25) << CustomersSorted[i].angle << endl;
    }
}

void TabuSearch::DataInitialization(std::string FilePath)
{
    ifstream InputFile;
    InputFile.open(FilePath);
    // count the number of nodes
    if(InputFile.is_open()){
        string str;
        int LinesCount = 0;
        while(!InputFile.eof()){
            getline(InputFile, str);
            LinesCount++;
        }
        if(LinesCount > 0){
            this->NodeCount = LinesCount;
            this->CustomerCount = LinesCount-1;
            InputFile.close();
        }
        else{
            cout << "The file is empty" << endl;
            exit(1);
        }
    }
    else{
        cout << "File could not be opened." << endl;
        exit(1);
    }
    // read date into map
    InputFile.open(FilePath);
    if(InputFile.is_open()){
        // define 2D dynamic array
        double** items = new double* [NodeCount];
        for(int i=0; i<NodeCount; i++){
            items[i] = new double[7];
        }
        while(!InputFile.eof()){
            for(int i=0; i<NodeCount; i++){
                for(int j=0; j<7; j++){
                   InputFile >> items[i][j];
                }
            }
        }
        DepotStart = new Depot((int)items[0][0], items[0][1], items[0][2], items[0][4], items[0][5]);
        NodeMap[-1] = DepotStart;
        for(int i=1; i<NodeCount; i++){
            Customer *c= new Customer((int)items[i][0],items[i][1],items[i][2],items[i][3],
                    items[i][4],items[i][5],items[i][6]);
            c->id = CustomerMap.size();
            CustomerMap[c->id] = c; // the customer id starts from 0
            NodeMap[c->id] = c;
             //initialise the tabu list with the value 0
        }
        // clean up
        for(int i=0; i<NodeCount; i++){
            delete []items[i];
        }
        delete [] items;
    }
    // set up the tabu list, and initial the value to 0
    attribute = new TabuAttribute* [CustomerCount];
    for(int i=0; i<CustomerCount; i++){
        attribute[i] = new TabuAttribute[VehicleCount];
    }
    for(int i=0; i<CustomerCount; i++){
        for(int j=0; j<VehicleCount; j++){
            TabuAttribute tabuAttribute(i,j);
            attribute[i][j] = tabuAttribute;
        }
    }
    //set up the arc attribute array
    ArcAttribute = new TabuArcAttribute* [CustomerCount];
    for(int i=0; i<CustomerCount; ++i){
        ArcAttribute[i] = new TabuArcAttribute[CustomerCount];
    }
    for(int i=0; i<CustomerCount; ++i){
        for(int j=0; j<CustomerCount; ++j){
            TabuArcAttribute tabuArcAttribute(i,j);
            ArcAttribute[i][j] = tabuArcAttribute;
        }
    }

    //dispaly the initial attribute data
    /*
    for(int i=0; i<CustomerCount; i++){
        for(int j=0; j<VehicleCount; j++){
            cout << "attribute: " << i << " & " << j << "  :"<< attribute[i][j].CustomerId()
                 << " " << attribute[i][j].VehicleId() << " " << attribute[i][j].VisitedTimes()
                 << " " << attribute[i][j].TabuStatus() << endl;
        }
    }
    */
}


void TabuSearch::VerifyDistanceCal()
{
    vector<Customer> CustomerList;
    vector<Path*> PathList;
    list<Path>::iterator it;


    //int CustomerId1[] = {42,43,15,87,57,41,22,74,73,21,26};
    //int CustomerId1[] = {83,45,61,16,99,6,95,97,13};
    int CustomerId1[] = {42,43,15};
    for(int i=0; i<3; i++){
        Customer c = *CustomerMap[CustomerId1[i]-1];
        cout << " X " << c.Xcoord() << "  Due date:  " <<  c.DueDate() << endl;
        CustomerList.push_back(c);
    }
    Path path1(*DepotStart, CustomerList);
    path1.SetId(1);
    cout << path1.DurationViolation() << endl;
    LocalSearch::AllPathList.push_back(path1);
    it = LocalSearch::AllPathList.end();
    --it;
    PathList.push_back(& *it);
    CustomerList.clear();

    int CustomerId2[] = {70,30,20,66,9,35,65,71,51,50,0};
    for(int i=0; i<10; i++){
        Customer c = *CustomerMap[CustomerId2[i]-1];
        CustomerList.push_back(c);
    }
    Path path2(*DepotStart, CustomerList);
    path1.SetId(2);
    LocalSearch::AllPathList.push_back(path2);
    it = LocalSearch::AllPathList.end();
    --it;
    PathList.push_back(& *it);
    CustomerList.clear();

    int CustomerId3[] = {28,1,69,76,53,40,2,13,95,97,58};
    for(int i=0; i<11; i++){
        Customer c = *CustomerMap[CustomerId3[i]-1];
        CustomerList.push_back(c);
    }
    Path path3(*DepotStart, CustomerList);
    path3.SetId(3);
    LocalSearch::AllPathList.push_back(path3);
    it = LocalSearch::AllPathList.end();
    --it;
    PathList.push_back(& *it);
    CustomerList.clear();

    int CustomerId4[] = {72,75,56,23,67,39,55,4,25,54};
    for(int i=0; i<10; i++){
        Customer c = *CustomerMap[CustomerId4[i]-1];
        CustomerList.push_back(c);
    }
    Path path4(*DepotStart, CustomerList);
    path4.SetId(4);
    LocalSearch::AllPathList.push_back(path4);
    it = LocalSearch::AllPathList.end();
    --it;
    PathList.push_back(& *it);
    CustomerList.clear();

    int CustomerId5[] = {68,80,24,29,79,78,34,81,33,3,77,12};
    for(int i=0; i<12; i++){
        Customer c = *CustomerMap[CustomerId5[i]-1];
        CustomerList.push_back(c);
    }
    Path path5(*DepotStart, CustomerList);
    path5.SetId(5);
    LocalSearch::AllPathList.push_back(path5);
    it = LocalSearch::AllPathList.end();
    --it;
    PathList.push_back(& *it);
    CustomerList.clear();

    int CustomerId6[] = {18,82,48,46,8,47,36,49,19,7};
    for(int i=0; i<10; i++){
        Customer c = *CustomerMap[CustomerId6[i]-1];
        CustomerList.push_back(c);
    }
    Path path6(*DepotStart, CustomerList);
    path6.SetId(6);
    LocalSearch::AllPathList.push_back(path6);
    it = LocalSearch::AllPathList.end();
    --it;
    PathList.push_back(& *it);
    CustomerList.clear();

    int CustomerId7[] = {92,98,14,44,38,86,16,61,85,91,100,37};
    for(int i=0; i<12; i++){
        Customer c = *CustomerMap[CustomerId7[i]-1];
        CustomerList.push_back(c);
    }
    Path path7(*DepotStart, CustomerList);
    path7.SetId(7);
    LocalSearch::AllPathList.push_back(path7);
    it = LocalSearch::AllPathList.end();
    --it;
    PathList.push_back(& *it);
    CustomerList.clear();

    int CustomerId8[] = {6,94,96,59,93,99,84,17,45,83,5,60,89};
    for(int i=0; i<13; i++){
        Customer c = *CustomerMap[CustomerId8[i]-1];
        CustomerList.push_back(c);
    }
    Path path8(*DepotStart, CustomerList);
    path8.SetId(8);
    LocalSearch::AllPathList.push_back(path8);
    it = LocalSearch::AllPathList.end();
    --it;
    PathList.push_back(& *it);
    CustomerList.clear();

    int CustomerId9[] = {52,88,62,11,64,63,90,32,10,31,27};
    for(int i=0; i<11; i++){
        Customer c = *CustomerMap[CustomerId9[i]-1];
        CustomerList.push_back(c);
    }
    Path path9(*DepotStart, CustomerList);
    path9.SetId(9);
    LocalSearch::AllPathList.push_back(path9);
    it = LocalSearch::AllPathList.end();
    --it;
    PathList.push_back(& *it);

    Solution solution(PathList, false);

    solution.DisplaySolution();
}

void TabuSearch::CleanUp()
{
    for(int i=0; i<CustomerCount; i++){
        delete [] attribute[i];
    }
    delete [] attribute;

    for(int i=0; i<CustomerCount; ++i){
        delete [] ArcAttribute[i];
    }
    delete [] ArcAttribute;

    delete DepotStart;

    LocalSearch::AllNeighbors.clear();
    LocalSearch::AllPathList.clear();
}







