#ifndef TABUSEARCH_H
#define TABUSEARCH_H
#include "dataClass.h"
#include "tabuAttribute.h"
#include <string>

class TabuSearch
{
private:
    static double epsilon; // is used for the comparison of two floating values
    int RandomNum; // the random number that determine the initial solution
    int NodeCount, CustomerCount, VehicleCount, CurrentFrequency;
    int IterNum, MaxIterNum, ConsIterNum, MaxConsIterNum;
    int TabuMemoryLength;
    int LocalSearchRule; // the rule 1 is relocate, rule 2 is the swap, rule 3 is the cross exchange
    double alpha, beta, gamma; //3 parameters in the objective function
    double ScalingFactor; // the factor used in the diversity strategy
    double delta; // factor used to modify the three parameters in the obj function
    double lambda; // the factor used to contol the intensity of the diversification
    double BestObjectiveValue; // the objective value of the best solution identified so far, this value used for apiration criterion, the calculation of objective value is the sum of travel distance and violation
    double ComputationTimeInMinute, ComputationTimeInSecond;
    int NumOfNeighbors; // the number of neighbor solutions generated in each iteration
    Solution FeasibleSolution; // the best feasible solution found
    double FeasibleSolutionCost;
    bool FeasibleSolutionFound;
    std::string FilePath;
    std::map<int, Node*> NodeMap;
    std::map<int, Customer*> CustomerMap; // CustomerList contains all the customers
    Depot *DepotStart;
    std::vector<Customer> CustomersSorted;
    //Depot *DepotEnd; for open vehicle routing, does not have end depot
    //std::vector<Solution> SolutionList;
    std::vector<Solution> SolutionList;
    static bool DataSortPredicate(const Customer& c1, const Customer& c2);
    void GetInitialSolution();
    void SortCustomers();
    void DataInitialization(std::string FilePath);
    void CheckCapacityCons();
    void DisplaySolution();
    bool DurationConstraint(Customer &InsertCustomer, int InsertPosition,std::vector<Customer> &CustomersVisited);
    void GetObjectiveValue(Solution& solution);
    Solution FirstImprovedNeigborhood(Solution CurrentSolution);//local search to find the first improvement neighborhood around the current solution
    void LocalSearchLoop(int IterationNum);
    bool CheckFeasibility(Solution& solution); // check the feasibility of a solution
    std::vector<NeighborSolution> GenerateNeighborSolutions(Solution CurrentSolution, int NumOfNeighbors); // generator a number of neighbor solutions
    void GetAllNeighborSolutions(Solution& CurrentSolution); // generate all neighbor solutions of the current solution

    NeighborSolution BestNeighbor(std::vector<NeighborSolution>& NeighborSolutions, Solution CurrentSolution); // find the best neighbor solution from the list of neighborhoods
    void DisplayNeighborSolution(std::vector<NeighborSolution>& NeighborSolutions);
    bool AcceptNeighbor(NeighborSolution& BestNeighborSolution);
    int GetFrequency(NeighborSolution& BestNeighborSolution, bool UpdateAttributeFrequency);
    void UpdateTabuList(NeighborSolution& BestNeighborSolution);

    void RunTabuSearch(int IterationNum);
    TabuAttribute** attribute ;
    void UpdateParameters(Solution& CurrentSolution);
    double GetPenalty(double cost, int MoveFrequency); // the penalty is used for the diversification
    void CleanUp();
    void VerifyDistanceCal();

    void FindBestInitial();
public:
    TabuSearch();
    void DisplayCustomerData();
    void RunModel();
};

#endif // TABUSEARCH_H
