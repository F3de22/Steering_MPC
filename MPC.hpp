#include <vector>
#include <fstream>
#include <tuple>
#include <limits.h>
#include <math.h>
#include "geometry.hpp"
#include <iostream>
#include <osqp.h>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>


using namespace std;

// MPC per il controllo dello sterzo
class MPC {
    public:
        MPC(); // costruttore dell'MPC

        //aggiorna A e B e discretizza con metodo Zero-Order Hold ottenendo Ad e Bd --> TODO: se si crea un oggetto MPC ad ogni passo allora inserirlo nel costruttore
        void updateDiscretization(double vx, double yaw_angle, double acc);

        // metodo per creare un vettore di coppie di valori (carico sulla ruota[N] + cornering stiffness[N/rad] + ) presi dal file
        vector<vector<double>>load_data();

        // metodo per il calcolo delle cornering stiffness 
        pair<double,double> load_transfer(double acceleration, const vector<vector<double>> &numeri);

        //metodo per il calcolo della funzione di costo che restituisce l'angolo di sterzo
        double compute(const Eigen::VectorXd& x0, vector<Point> waypoints);

    private:
        void updateDiscretization(double vx);

        //matrici da aggiornare ad ogni passo
        Eigen::MatrixXd A, B; // matrici continue
        Eigen::MatrixXd Ad, Bd; // matrici discrete 
        
        double la = 0.792; // distanza tra semi-asse anteriore e centro di massa
        double lb = 0.758; // distanza tra semi-asse posteriore e centro di massa
        double Iz = 98.03; // momento di inerzia
        double m = 320; // massa del veicolo
        double Ycm = 0.362; // altezza dell centro di massa del veicolo
        int nx = 5; // numero di stati
        int nu = 1; // numero di input
        int op = 20; // orizzonte di predizione --> si predice il sistema per i successivi dt*op secondi, cioè dt*op*v metri
        int oc = 10; // orizzonte di controllo (oc <= op)
        double dt = 0.02; // passo di campionamento
        Eigen::MatrixXd Q;   // peso stato (nx x nx) --> fare una funzione setWeight() se si vuole cambiare i pesi in certe situazioni
        Eigen::MatrixXd R;   // peso input (nu x nu)
        Eigen::MatrixXd R_delta;  //peso sulla variazione di sterzo
        Eigen::VectorXd xmin, xmax;  // limiti stato (nx)
        Eigen::VectorXd umin, umax;  // limiti input (nu)
        Eigen::VectorXd u_prev; // per warm start: mi salvo i controlli ottimi calcolati al passo precedente

        OsqpEigen::Solver solver;
        bool solver_initialized;

        std::vector<std::vector<double>> numeri;
};
