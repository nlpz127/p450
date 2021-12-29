#include <iostream>
using namespace std;
#include <iomanip>

const float EPSINON = 0.00001;

int main(){
    
    cout.setf(ios::fixed);
    cout << setprecision(2);
    cout.setf(ios::left);
    cout.setf(ios::showpoint);

    float a;
    cin >> a;

    
    cout << a << endl;
    cout << abs(a+1.5) << endl;

    if(abs(a+1.5) < EPSINON || abs(a+1.5) == EPSINON){
        cout << "equal" << endl;
    }
    else{
        cout << "No!" << endl;
    }

    return 0;
}
