#include <iostream>
#include <math.h>
#include <time.h>

#include "tabusearch.h"

using namespace std;

int main()
{

    //String FilePath = "C:/Eclipse Java/eclipse/Workspace/VRPTW/data/Instances2.txt";
    //String FilePath = "/Users/fulin/Documents/Java/Workspace/VRPTW/data/Instances2.txt";

    TabuSearch model;
    model.RunModel();
    //model.DisplayCustomerData();

    /*
    srand(time(0));
    //test the random number
    for(int i=0; i< 1; i++){

        int j = rand()%100;
        cout<<j<<endl;
    }
    */
    return 0;

}

