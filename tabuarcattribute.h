#ifndef TABUARCATTRIBUTE
#define TABUARCATTRIBUTE

class TabuArcAttribute
{
private:
    int CustomerId0;
    int CustomerDescId0; // the id of the first customer's descendant
    int TabuStatus0;

public:
    TabuArcAttribute() : CustomerId0(0), CustomerDescId0(0), TabuStatus0(0){}

    TabuArcAttribute(int CustomerId, int CustomerDescId){
        this->CustomerDescId0 = CustomerId;
        this->CustomerDescId0 = CustomerDescId;
        this->TabuStatus0 = 0;
    }

    void SetTabuStatus(int TabuStatus){this->TabuStatus0 = TabuStatus;}

    int CustomerId(){return this->CustomerId0;}
    int CustomerDescId(){return this->CustomerDescId0;}
    int TabuStatus(){return this->TabuStatus0;}
};






#endif // TABUARCATTRIBUTE

