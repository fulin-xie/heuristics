#ifndef TABUATTRIBUTE
#define TABUATTRIBUTE

class TabuAttribute
{
private:
    int CustomerId0;
    int PathId0;
    int TabuStatus0;
    int VisitedTimes0; //the total number of times the attribute has been vistied during the searching process
public:
    TabuAttribute(int CustomerId, int PathId){
        this->CustomerId0 = CustomerId;
        this->PathId0 = PathId;
        this->TabuStatus0 = 0;
        this->VisitedTimes0 = 0;
    }
    TabuAttribute() : CustomerId0(0), PathId0(0), TabuStatus0(0), VisitedTimes0(0){}

    int CustomerId(){return this->CustomerId0;}
    int VehicleId(){return this->PathId0;}
    int TabuStatus(){return this->TabuStatus0;}
    int VisitedTimes(){return this->VisitedTimes0;}

    void SetTabuStatus(int TabuStatus){this->TabuStatus0 = TabuStatus;}
    void SetVisitedTimes(int VisitedTimes){this->VisitedTimes0 = VisitedTimes;}
};

#endif // TABULIST

