#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

#include <vector>
#include <list>
#include "dataClass.h"
#include "tabuAttribute.h"

class LocalSearch
{
private:
    static double epsilon;
    int CustomerMoved0; // the id of the customer that is moved during the neighborhood search
    int PathOut0; // the id of the vehicle that has been moved out one customer
    int PathIn0; // the id of the vehicle that has been inserted the moved customer

    double GetNewDriveDistance(Path PathToChange, Depot& DepotStart, std::vector<Customer>& CustomerList, bool CustomerMoveOut,
                               Customer& CustomerToChange, int PositionIndex);
public:
    static std::vector<NeighborSolution> AllNeighbors;
    static std::list<Path> AllPathList; // the list of all paths that have been generated
    static std::list<Path> TemPathList;

    LocalSearch(){} //: CustomerMoved0(0), PathOut0(0), PathIn0(0){}
    int CustomerMovedId(){return this->CustomerMoved0;}
    int PathOutId(){return this->PathOut0;}
    int PathInId(){return this->PathIn0;}
    // the relocate operator of the local search
    void relocate(Solution CurrentSolution, Depot DepotStart, double BestObjectiveValue,
                                                    double alpha, double beta, double gamma, TabuAttribute** attribute );
    // the cross exchange operator
    //void CrossExchange(Solution CurrentSolution, Depot DepotStart, double alpha, double beta, double gamma);
    // get the new customer list for the cross exchange operator
    std::vector<Customer> GetNewCustomerList(std::vector<Customer> CustomerListOne, std::vector<Customer> CustomerListTwo,
                                                int StartOne, int StartTwo, int EndOne, int EndTwo);
    // swap the positions of two selected customers
    void swap(Solution CurrentSolution, Depot DepotStart, double alpha, double beta, double gamma);

    void TwoOptAsterisk(Solution CurrentSolution, Depot DepotStart, double alpha, double beta, double gamma);
};

#endif // LOCALSEARCH_H
