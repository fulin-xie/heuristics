#include <vector>
#include <random>
#include <iostream>

#include "localSearch.h"

using namespace std;

vector<NeighborSolution> LocalSearch::AllNeighbors;
list<Path> LocalSearch::AllPathList;
list<Path> LocalSearch::TemPathList;
double LocalSearch::epsilon = 0.00001;

//generate all neibor solutions around the current solution
void LocalSearch::relocate(Solution CurrentSolution,Depot DepotStart, double BestObjectiveValue,
                                                      double alpha, double beta, double gamma, TabuAttribute** attribute)
{
    AllNeighbors.clear(); // clear the vector of all neighborhood solutions
    TemPathList.clear();

    int PathCount = CurrentSolution.NumOfPaths();
    vector<Path*> PathListCurrent = CurrentSolution.PathList();

    for(int i=0; i<PathCount; ++i){ // loop 1, select a path in which the customer will be removed
        Path PathOut = *PathListCurrent[i]; //the path in which the customer will remove
        vector<Customer> CustomerListOfPathOut = PathOut.CustomersList();
        int CustomerCount = CustomerListOfPathOut.size();

        for(int j=0; j<CustomerCount; j++){ // loop 2, select a customer to remove from the path selected
            Customer CustomerOut = CustomerListOfPathOut[j]; // the customer that will be moved out

            double NewLoad1, NewDriveDistance1;
            NewLoad1 = PathOut.TotalLoad() - CustomerOut.demand();

            Depot DepotStartOut =  PathOut.DepotStart;
            vector<Customer> NewCustomerListOut = CustomerListOfPathOut;

            NewDriveDistance1 = GetNewDriveDistance(PathOut, DepotStartOut, NewCustomerListOut ,true, CustomerOut, j);

            NewCustomerListOut.erase(NewCustomerListOut.begin()+j); //delete the removed customer

            Path NewPathOut(DepotStartOut, NewCustomerListOut, NewDriveDistance1, NewLoad1);
            //Path NewPathOut(DepotStart, NewCustomerListOfPathOut);
            NewPathOut.SetId(PathOut.id());
            /*
            double NewLoad3, NewDriveDistance3;
            NewLoad3 = NewPathOut.TotalLoad();
            NewDriveDistance3 = NewPathOut.DriveDistance();
            */
            for(int m=0; m<PathCount; m++){ // loop 3, select the path to insert the customer
                if(m != i){
                    vector<Path*> NewPathList = PathListCurrent;
                    NewPathList.erase(NewPathList.begin()+i); // delete the pathout from the NewPathList

                    Path PathIn = *PathListCurrent[m]; //the path that will be inserted the customer
                    vector<Customer> CustomerListOfPathIn = PathIn.CustomersList();
                    if(m < i){
                        NewPathList.erase(NewPathList.begin()+m);
                    }
                    else{
                        NewPathList.erase(NewPathList.begin() + (m-1));
                    }
                    //determine the best insert positon wich minimise the objective function
                    Path NewPathIn; // the path with the best insert position
                    if(CustomerListOfPathIn.empty()==true){
                        vector<Customer> NewCustomersList = CustomerListOfPathIn;
                        NewCustomersList.push_back(CustomerOut);
                        Path path(DepotStart, NewCustomersList);
                        NewPathIn = path;
                        NewPathIn.SetId(PathIn.id());
                    }

                    else{
                        //determine the insert position, possible paths
                        Path* PossiblePath = new Path[CustomerListOfPathIn.size()+1];
                        for(int n=0; n<(int)CustomerListOfPathIn.size()+1; n++ ){
                            vector<Customer> NewCustomersList = CustomerListOfPathIn;

                            double NewDriveDistance2, NewLoad2;
                            Depot NewDepotStart = PathIn.DepotStart;
                            NewDriveDistance2 = GetNewDriveDistance(PathIn, NewDepotStart,NewCustomersList ,false, CustomerOut,n);
                            NewLoad2 = PathIn.TotalLoad() + CustomerOut.demand();

                            vector<Customer>::iterator it = NewCustomersList.begin();
                            advance(it,n);
                            NewCustomersList.insert(it,CustomerOut);
                            Path path(NewDepotStart, NewCustomersList, NewDriveDistance2, NewLoad2);
                            //Path path(DepotStart, NewCustomersList);
                            PossiblePath[n] = path;
                            /*
                            //test
                            //get a copy of the NewPathList
                            vector<Path> NewPathListCopy = NewPathList;
                            NewPathIn = path;
                            NewPathIn.SetId(PathIn.id());
                            NewPathIn.GetObjectiveValue(alpha, beta, gamma);
                            NewPathListCopy.push_back(NewPathIn);
                            NewPathListCopy.push_back(NewPathOut);
                            NeighborSolution NewNeighbor(NewPathListCopy, NewPathOut.id(), NewPathIn.id(), CustomerOut.id);
                            NewNeighbor.GetObjectiveValue(alpha, beta, gamma);
                            AllNeighbors.push_back(NewNeighbor);
                            */
                            /*
                            double NewLoad4, NewDriveDistance4;
                            NewDriveDistance4 = path.DriveDistance();
                            NewLoad4 = path.TotalLoad();
                            */
                        }

                        double MinObjValue = 100000000000; //the path with the min objective value
                        int PathWithMinObjValue;
                        for(int n=0; n<(int)CustomerListOfPathIn.size()+1; n++){
                            PossiblePath[n].GetObjectiveValue(alpha, beta, gamma);
                            /* display information of possible paths
                            Path pathcopy = PossiblePath[n];
                            pathcopy.DisplayInfo();
                            std::cout << "Objective value:  " << pathcopy.ObjectiveValue()<<std::endl;
                            */
                            if((MinObjValue-PossiblePath[n].ObjectiveValue()) > epsilon ){
                                MinObjValue = PossiblePath[n].ObjectiveValue();
                                PathWithMinObjValue = n;
                            }
                        }
                        NewPathIn = PossiblePath[PathWithMinObjValue];
                        NewPathIn.SetId(PathIn.id());
                        //clean up
                        delete []PossiblePath;
                    }

                    //calculate solution distance and violations
                    Solution TemSolution(NewPathList, false);
                    double DriveDistance = TemSolution.TotalDriveDistance()
                            + NewPathIn.DriveDistance() + NewPathOut.DriveDistance();
                    double CapacityViolation = TemSolution.TotalCapacityViolation()
                            + NewPathIn.CapacityViolation() + NewPathOut.CapacityViolation();
                    double DurationViolation = TemSolution.TotalDurationViolation()
                            + NewPathIn.DurationViolation() + NewPathOut.DurationViolation();
                    double TimeWindowViolation = TemSolution.TotalTimeWindowViolation()
                            + NewPathIn.TimeWindowViolation() + NewPathOut.TimeWindowViolation();

                    TemPathList.push_back(NewPathOut);
                    TemPathList.push_back(NewPathIn);
                    list<Path>::iterator it = TemPathList.end();
                    --it;
                    NewPathList.push_back(& *it);
                    --it;
                    NewPathList.push_back(& *it);

                    NeighborSolution NewNeighbor (NewPathList, DriveDistance, CapacityViolation, DurationViolation,
                                   TimeWindowViolation, NewPathOut.id(), CustomerOut.id, NewPathIn.id(), -1, true);
                    //NeighborSolution NewNeighbor (NewPathList, NewPathOut.id(), NewPathIn.id(), CustomerOut.id, false);
                    //NewNeighbor.UpdateViolationValues(CurrentSolution,PathOut,PathIn,NewPathOut,NewPathIn);
                    NewNeighbor.GetObjectiveValue(alpha, beta, gamma);
                    AllNeighbors.push_back(NewNeighbor);

                    /*
                    // the solution is better than the best solution found in the previous iterations
                    double NormalisedObjValue = NewNeighbor.TotalDriveDistance() + NewNeighbor.TotalViolation();
                    if(NormalisedObjValue < BestObjectiveValue){
                        NewNeighbor.SetFirstImprovedNeighbor(true);
                        goto FoundFirstImproved;
                    }

                    //check if the solution is an improved and non-tabu solution
                    if((attribute[CustomerOut.id][PathIn.id()].TabuStatus()==0) &&
                        (NewNeighbor.ObjectiveValue() < CurrentSolution.ObjectiveValue())){
                        NewNeighbor.SetFirstImprovedNeighbor(true);
                        goto FoundFirstImproved;
                    }
                    */
                }
            }
        }
    }
    FoundFirstImproved:
    int size = 0;
    size = AllPathList.size();
    size = size;
}


void LocalSearch::swap(Solution CurrentSolution, Depot DepotStart,double alpha, double beta, double gamma)
{
    AllNeighbors.clear();
    TemPathList.clear();

    int PathCount = CurrentSolution.NumOfPaths();
    vector<Path*> PathListCurrent = CurrentSolution.PathList(); // the list of paths of the current solution

    for(int i=0; i<(PathCount-1); i++){ // loop1, select a path that will be involved into the cross exchange operator
        Path PathOne = *PathListCurrent[i];
        vector<Customer> CustomerListOne = PathOne.CustomersList();

        for(int j=i+1; j<PathCount; j++){ // loop2, select a different path as the second path involved in the cross-exchange
            Path PathTwo = *PathListCurrent[j];
            vector<Customer> CustomerListTwo = PathTwo.CustomersList();
            vector<Path*> NewPathList = PathListCurrent;
            NewPathList.erase(NewPathList.begin()+i); // remove this path from the solution
            NewPathList.erase(NewPathList.begin()+(j-1)); // remove the second path from the solution

            //calculate solution distance and violations
            Solution TemSolution(NewPathList, false);
            double TemDriveDistance = TemSolution.TotalDriveDistance();
            double TemCapacityViolation = TemSolution.TotalCapacityViolation();
            double TemDurationViolation = TemSolution.TotalDurationViolation();
            double TemTimeWindowViolation = TemSolution.TotalTimeWindowViolation();

            if(CustomerListOne.empty() == false && CustomerListTwo.empty() == false){
                for(int m=0; m<(int)CustomerListOne.size(); m++){ //loop3 select a customer from the path one
                    Customer CustomerOne = CustomerListOne[m];
                    vector<Customer> NewCusListOne = CustomerListOne;
                    NewCusListOne.erase(NewCusListOne.begin()+m);

                    for(int n=0; n<(int)CustomerListTwo.size();n++){ //loop 4 select another customer from the path two
                        Customer CustomerTwo = CustomerListTwo[n];
                        vector<Customer> NewCusListTwo = CustomerListTwo;
                        NewCusListTwo.erase(NewCusListTwo.begin()+n);

                        //get copy of NewPathList and NewCusListOne
                        vector<Path*> NewPathListCopy = NewPathList;
                        vector<Customer> NewCusListOneCopy = NewCusListOne;

                        NewCusListOneCopy.insert(NewCusListOneCopy.begin()+m, CustomerTwo);
                        NewCusListTwo.insert(NewCusListTwo.begin()+n, CustomerOne);

                        Path NewPathOne(DepotStart ,NewCusListOneCopy);
                        NewPathOne.SetId(PathOne.id());
                        Path NewPathTwo(DepotStart ,NewCusListTwo);
                        NewPathTwo.SetId(PathTwo.id());

                        double DriveDistance = TemDriveDistance
                                + NewPathOne.DriveDistance() + NewPathTwo.DriveDistance();
                        double CapacityViolation = TemCapacityViolation
                                + NewPathOne.CapacityViolation() + NewPathTwo.CapacityViolation();
                        double DurationViolation = TemDurationViolation
                                + NewPathOne.DurationViolation() + NewPathTwo.DurationViolation();
                        double TimeWindowViolation = TemTimeWindowViolation
                                + NewPathOne.TimeWindowViolation() + NewPathTwo.TimeWindowViolation();

                        TemPathList.push_back(NewPathOne);
                        TemPathList.push_back(NewPathTwo);
                        list<Path>::iterator it = TemPathList.end();
                        --it;
                        NewPathListCopy.push_back(& *it);
                        --it;
                        NewPathListCopy.push_back(& *it);

                        NeighborSolution NewNeighbor(NewPathListCopy, DriveDistance,CapacityViolation,DurationViolation,
                            TimeWindowViolation,PathOne.id(),CustomerOne.id, PathTwo.id(), CustomerTwo.id, true);
                        //NeighborSolution NewNeighbor(NewPathListCopy, PathOne.id(), PathTwo.id(), CustomerOne.id, false);

                        NewNeighbor.GetObjectiveValue(alpha, beta, gamma);
                        AllNeighbors.push_back(NewNeighbor);
                    }
                }
            }
        }
    }
    int size = 0;
    size = TemPathList.size();
    size = size;
}


// the 2_opt* operator, it swaps two tails of two route,
void LocalSearch::TwoOptAsterisk(Solution CurrentSolution, Depot DepotStart, double alpha, double beta, double gamma)
{
    AllNeighbors.clear(); // clear the vector
    TemPathList.clear();
    int PathCount = CurrentSolution.NumOfPaths();
    vector<Path*> PathListCurrent = CurrentSolution.PathList();

    for(int i=0; i<(PathCount-1); i++){ //loop1, select a path from the path list of the current solution
        Path PathOne = *PathListCurrent[i];
        vector<Customer> CustomerListOne = PathOne.CustomersList();

        for(int j=i+1; j<PathCount; j++){//LOOP2, select the second path from the path list
            Path PathTwo = *PathListCurrent[j];
            vector<Customer> CustomerListTwo = PathTwo.CustomersList();
            vector<Path*> NewPathList = PathListCurrent;
            NewPathList.erase(NewPathList.begin()+i); // remove the first path from the list
            NewPathList.erase(NewPathList.begin()+(j-1)); // remove the second path from the solution

            if(CustomerListOne.empty()==false && CustomerListTwo.empty()==false){ // both of two selected paths are empty
                for(int m=0; m<(int)CustomerListOne.size(); m++){ //loop3 select a customer from the path
                    vector<Customer> Segment1, Segment2;
                    for(int p=0; p<m; p++){
                        Segment1.push_back(CustomerListOne[p]);
                    }
                    for(int q=m; q<(int)CustomerListOne.size(); q++){
                        Segment2.push_back(CustomerListOne[q]);
                    }

                    for(int n=0; n<(int)CustomerListTwo.size(); n++){ // loop2 select a node place to split the customer list
                        vector<Customer> Segment3, Segment4;
                        for(int p=0; p<n; p++){
                            Segment3.push_back(CustomerListTwo[p]);
                        }
                        for(int q=n; q<(int)CustomerListTwo.size();q++){
                            Segment4.push_back(CustomerListTwo[q]);
                        }
                        vector<Customer> NewCustomerList1, NewCustomerList2;
                        NewCustomerList1 = Segment1;
                        NewCustomerList1.insert(NewCustomerList1.end(), Segment4.begin(), Segment4.end());
                        NewCustomerList2 = Segment3;
                        NewCustomerList2.insert(NewCustomerList2.end(), Segment2.begin(), Segment2.end());

                        vector<Path*> NewPathListCopy = NewPathList;

                        Path NewPathOne(DepotStart, NewCustomerList1);
                        NewPathOne.SetId(PathOne.id());
                        Path NewPathTwo(DepotStart, NewCustomerList2);
                        NewPathTwo.SetId(PathTwo.id());

                        TemPathList.push_back(NewPathOne);
                        TemPathList.push_back(NewPathTwo);

                        list<Path>::iterator it = TemPathList.end();
                        --it;
                        NewPathListCopy.push_back(& *it);
                        --it;
                        NewPathListCopy.push_back(& *it);
                        NeighborSolution NewNeighbor(NewPathListCopy, PathOne.id(),CustomerListOne[m].id,
                                                     PathTwo.id(), CustomerListTwo[n].id, false);
                        NewNeighbor.GetObjectiveValue(alpha, beta, gamma);
                        AllNeighbors.push_back(NewNeighbor);
                    }
                }
            }
        }
    }
}

/*

void LocalSearch::CrossExchange(Solution CurrentSolution, Depot DepotStart,
                                                                double alpha, double beta, double gamma)
{
    AllNeighbors.clear();

    int SegmentMaxLength = 2;
    int PathCount = CurrentSolution.NumOfPaths();
    vector<Path> PathListCurrent = CurrentSolution.PathList(); // the list of paths of the current solution

    for(int i=0; i<(PathCount-1); i++){ // loop1, select a path that will be involved into the cross exchange operator
        Path PathOne = PathListCurrent[i];
        vector<Customer> CustomerListOne = PathOne.CustomersList();

        for(int j=i+1; j<PathCount; j++){ // loop2, select a different path as the second path involved in the cross-exchange
            Path PathTwo = PathListCurrent[j];
            vector<Customer> CustomerListTwo = PathTwo.CustomersList();
            vector<Path> NewPathList = PathListCurrent;
            NewPathList.erase(NewPathList.begin()+i); // remove this path from the solution
            NewPathList.erase(NewPathList.begin()+(j-1)); // remove the second path from the solution

            for(int k=0; k<(int)CustomerListOne.size(); k++){ // loop3, select the start of the first segment

                for(int l=0; l<(int)CustomerListTwo.size(); l++){ // loop 4, select the start of the second segment

                    for(int m=k; (m<(int)CustomerListOne.size())&&(m<(k+SegmentMaxLength)) ; m++){// loop 5, select the end of the first segment

                        for(int n=l; (n<(int)CustomerListTwo.size())&&(n<(l+SegmentMaxLength)); n++){ // loop 6, select the end of the seconde segment
                            vector<Customer> NewCusListOne, NewCusListTwo; // the new customer list after the cross exchange
                            //constrcut the Customer List
                            NewCusListOne = GetNewCustomerList(CustomerListOne, CustomerListTwo, k, l, m, n);
                            NewCusListTwo = GetNewCustomerList(CustomerListTwo, CustomerListOne, l, k, n, m);

                            //get a copy of the NewPathList
                            vector<Path> NewPathListCopy = NewPathList;

                            Path NewPathOne(DepotStart, NewCusListOne);
                            NewPathOne.SetId(PathOne.id());
                            Path NewPathTwo(DepotStart, NewCusListTwo);
                            NewPathTwo.SetId(PathTwo.id());
                            //insert new pathes into the path list
                            NewPathListCopy.push_back(NewPathOne);
                            NewPathListCopy.push_back(NewPathTwo);
                            NeighborSolution NewNeighbor(NewPathListCopy,PathOne.id(),PathTwo.id(),CustomerListOne[k].id, false);
                            NewNeighbor.GetObjectiveValue(alpha, beta, gamma);
                            AllNeighbors.push_back(NewNeighbor);
                        }
                    }
                }
            }
        }
    }
    int size = 0;
    size = AllNeighbors.size();
}
*/

vector<Customer> LocalSearch::GetNewCustomerList(std::vector<Customer> CustomerListOne, std::vector<Customer> CustomerListTwo,
                                                 int StartOne, int StartTwo, int EndOne, int EndTwo)
{
    vector<Customer> NewCusList; // the new customer list after the cross exchange
    // constrcut the Customer List
    // insert customers before the StartOne into the new customer list
    for(int i=0; i<StartOne; i++){
        Customer c = CustomerListOne[i];
        NewCusList.push_back(c);
    }
    // insert the exchange segment into the customer list
    for(int j=StartTwo; j<=EndTwo; j++){
        Customer c = CustomerListTwo[j];
        NewCusList.push_back(c);
    }
    //insert customers after the EndOne into the new customer list
    if(EndOne < (int)(CustomerListOne.size()-1)){ // if there are still customers left after the removed segment
        for(int m=(EndOne+1); m<(int)CustomerListOne.size(); m++){
        Customer c= CustomerListOne[m];
        NewCusList.push_back(c);
        }
    }
    return NewCusList;
}



double LocalSearch::GetNewDriveDistance(Path PathToChange, Depot& DepotStart,vector<Customer>& CustomerList,
                                        bool CustomerMoveOut,Customer& CustomerToChange, int PositionIndex)
{
    double NewDriveDistance;

    if(CustomerList.size()==1 && CustomerMoveOut == true){ // move out
        // customer removed from the path, otherwise customer is inserted into the path
        NewDriveDistance = 0;
        DepotStart.DriveTimeToNextNode = 0;
    }
    else if(CustomerList.empty()==true && CustomerMoveOut == false){ // move in
        double distance = DepotStart.TimeToNode(CustomerToChange);
        DepotStart.DriveTimeToNextNode = distance;
        NewDriveDistance = distance*2;
    }
    else{
        double distance1, distance2, distance3;
        double CurrentDriveDistance = PathToChange.DriveDistance();

        if(PositionIndex == 0){
            Customer BackCustomer;
            if(CustomerMoveOut==true){ // move out
                Customer c = CustomerList[PositionIndex + 1]; // the customer after the removed customer
                BackCustomer = c;
            }
            else{ // insert
                Customer c = CustomerList[PositionIndex];
                BackCustomer = c;
            }
            distance1 = DepotStart.TimeToNode(CustomerToChange);
            distance2 = CustomerToChange.TimeToNode(BackCustomer);
            distance3 = DepotStart.TimeToNode(BackCustomer);
            if(CustomerMoveOut==true){
                DepotStart.DriveTimeToNextNode = distance3;
            }
            else{ //insert
                DepotStart.DriveTimeToNextNode = distance1;
                CustomerToChange.DriveTimeToNextNode = distance2;
            }
        }
        else if((CustomerMoveOut==true && PositionIndex==int(CustomerList.size()-1))
                                || (CustomerMoveOut==false && PositionIndex==int(CustomerList.size()))){ // the end of the list
            Customer FrontCustomer = CustomerList[PositionIndex-1];
            distance1 = FrontCustomer.TimeToNode(CustomerToChange);
            distance2 = CustomerToChange.TimeToNode(DepotStart);
            distance3 = FrontCustomer.TimeToNode(DepotStart);
            if(CustomerMoveOut==true){
                FrontCustomer.DriveTimeToNextNode = distance3;
            }
            else{ //insert
                FrontCustomer.DriveTimeToNextNode = distance1;
                CustomerToChange.DriveTimeToNextNode = distance2;
            }
            CustomerList[PositionIndex-1] = FrontCustomer;
        }
        else{
            Customer FrontCustomer = CustomerList[PositionIndex-1];
            Customer BackCustomer;
            if(CustomerMoveOut==true){
                Customer c = CustomerList[PositionIndex+1];
                BackCustomer = c;
            }
            else{
                Customer c = CustomerList[PositionIndex];
                BackCustomer = c;
            }
            distance1 = FrontCustomer.TimeToNode(CustomerToChange);
            distance2 = CustomerToChange.TimeToNode(BackCustomer);
            distance3 = FrontCustomer.TimeToNode(BackCustomer);
            if(CustomerMoveOut==true){
                FrontCustomer.DriveTimeToNextNode = distance3;
            }
            else{ //insert
                FrontCustomer.DriveTimeToNextNode = distance1;
                CustomerToChange.DriveTimeToNextNode = distance2;
            }
            CustomerList[PositionIndex-1] = FrontCustomer;
        }
        if(CustomerMoveOut ==true){
            NewDriveDistance = CurrentDriveDistance - distance1 - distance2 + distance3;
        }
        else{ //the customer is inserted into the path
            NewDriveDistance = CurrentDriveDistance - distance3 + distance1 + distance2;
        }
    }
    return NewDriveDistance;
}


















