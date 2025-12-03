// expected output
//t=10 | meas=9.80 | est_pos=9.93 | est_vel=1.01
using namespace std;


#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include "Eigen/Dense"




// Function 1: Create and return a list (vector)
vector<float> makeList() {
    vector<float> nums;
    return nums;  // return the list
}

// Function 3: Append a single new item
void addItem(vector<float>& nums, float value) {
    nums.push_back(value);
}

void read_record(vector<float>& time_steps, vector<float>& positions)
{
    fstream fin;
    fin.open("measurements.csv", ios::in);

    vector<string> row;
    string line;

    while (getline(fin, line))
    {
        row.clear();
        stringstream ss(line);
        string word;

        while (getline(ss, word, ',')) {
            row.push_back(word);
        }

        if (row.size() < 2) continue;      // safety
        if (!isdigit(row[0][0])) continue; // skip header

        addItem(time_steps, stof(row[0]));
        addItem(positions,  stof(row[1]));
    }

    fin.close();   // <-- THIS MUST BE INSIDE THE FUNCTION
}                  // -------------------------------------








void kalman_filter(vector<float>& time_steps, vector<float>& positions)
{
    // DEFINING VARIABLES AND CONSTANTS

    Eigen::VectorXd x(2);
    Eigen::MatrixXd A(2, 2);
    A << 1, 1,
         0, 1;

    Eigen::Matrix2d Q;
    Q << 0.001, 0,
         0,     0.001;

    Eigen::RowVector2d H;   // 1×2 matrix
    H << 1, 0;

    float R = 0.04;

    Eigen::Matrix2d P;
    P << 1000, 0,
         0,    1000;

    Eigen::Matrix2d I;
    I.setIdentity();

    // INITIAL STATE
    x(0) = positions[0];   // first measured position
    x(1) = 0.0;            // assume starting at rest

    // MAIN LOOP: process measurements 1..N-1
    for (int i = 1; i < positions.size(); i++) {

        // --- dt from timestamps (here it will always be 1, but we do it properly) ---
        double dt = time_steps[i] - time_steps[i - 1];
        A(0, 1) = dt;   // [1 dt; 0 1]

        // --- PREDICTION ---
        Eigen::VectorXd x_pred = A * x;
        Eigen::Matrix2d P_pred = A * P * A.transpose() + Q;

        // --- MEASUREMENT UPDATE ---
        double z = positions[i];      // scalar measurement
        double y = z - x_pred(0);     // innovation

        double S = H * P_pred * H.transpose() + R;
        Eigen::Vector2d K = P_pred * H.transpose() * (1.0 / S);

        x = x_pred + K * y;
        P = (I - K * H) * P_pred;

        cout << "t=" << time_steps[i]
             << " | meas=" << z
             << " | est_pos=" << x(0)
             << " | est_vel=" << x(1) << "\n";
    }

    // FINAL ONE-STEP-AHEAD PREDICTION (t = last_time + dt)
    Eigen::Vector2d x_final_pred = A * x;

    cout << "\n=== Final Prediction (one step ahead) ===\n";
    cout << "Predicted next position = " << x_final_pred(0) << "\n";
    cout << "Predicted next velocity = " << x_final_pred(1) << "\n";
}


int main() {
    vector<float> time_steps = makeList();
    vector<float> positions = makeList();
    read_record(time_steps, positions);



    kalman_filter(time_steps, positions);
    
    return 0;
}
