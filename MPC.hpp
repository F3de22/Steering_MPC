#include <vector>
#include <fstream>
#include <tuple>
#include <limits.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <osqp.h>
#include <mpc/LMPC.hpp>

using namespace std;

struct MPC_config {
    int    nx;           // numero di stati
    int    nu;           // numero di input
    int    op;           // orizzonte di predizione
    int    oc;           // orizzonte di controllo (oc <= op)
    double dt;           // passo di campionamento
    Eigen::MatrixXd Q;   // peso stato (Nx x Nx)
    Eigen::MatrixXd R;   // peso input (Nu x Nu)
    Eigen::VectorXd xmin, xmax;  // limiti stato (Nx)
    Eigen::VectorXd umin, umax;  // limiti input (Nu)
};

// MPC per il controllo dello sterzo
class MPC {
    public:
        MPC(double a, double b, double Iz, double mass, double Ycm, const MPC_config& config); // costruttore dell'MPC


        void discretizeEulero(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double dt, Eigen::MatrixXd& Ad, Eigen::MatrixXd& Bd);

        // metodo per creare un vettore di coppie di valori (carico sulla ruota[N] + cornering stiffness[N/rad] + ) presi dal file
        vector<vector<double>>load_data();

        // metodo per il calcolo delle cornering stiffness 
        pair<double,double> load_transfer(double acceleration, const vector<vector<double>> &numeri);

    private:
        void updateDiscretization(double vx);

        MPC_config cfg;
        Eigen::MatrixXd A, B, Bdist;
        Eigen::MatrixXd Ad, Bd, Bdd;
        double la; // distanza tra semi-asse anteriore e centro di massa
        double lb; // distanza tra semi-asse posteriore e centro di massa
        double Iz; // momento di inerzia
        double m; // massa del veicolo
        double Ycm; // altezza dell centro di massa del veicolo    

        // TODO: aggiungi strutture per OSQP (matrici QP, solver, ecc.)
};
