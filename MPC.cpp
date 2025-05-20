#include "MPC.hpp"

MPC::MPC(double a, double b, double Iz, double mass, double ycm, const MPC_config& config):
    la(a),
    lb(b),
    m(mass),
    Iz(Iz),
    Ycm(ycm),
    cfg(config){}












//discretizzazione con metodo Zero-Order Hold
void MPC::discretizeEulero(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, double dt, Eigen::MatrixXd& Ad, Eigen::MatrixXd& Bd){
    int n = A.rows();
    int m = B.cols();

    Eigen::MatrixXd M(n + m, n + m);
    M.setZero();
    M.topLeftCorner(n, n) = A * dt;
    M.topRightCorner(n, m) = B * dt; // M--> matrice con in alto a sx A*dt e in alto a dx B*dt

    Eigen::MatrixXd expM = M.exp();

    Ad = expM.topLeftCorner(n, n);
    Bd = expM.topRightCorner(n, m);
}

void MPC::updateDiscretization(double vx) {
        A = Eigen::MatrixXd::Zero(cfg.nx, cfg.nx);
        B = Eigen::MatrixXd::Zero(cfg.nx, cfg.nu);
        Bdist = Eigen::MatrixXd::Zero(cfg.nx, 1);

        // TODO: inserire il modello continuo in funzione di vx

        int n = cfg.nx;
        int m = cfg.nu;

        Eigen::MatrixXd M(n + m, n + m);
        M.setZero();
        M.topLeftCorner(n, n) = A * cfg.dt;
        M.topRightCorner(n, m) = B * cfg.dt;

        Eigen::MatrixXd expM = M.exp();

        Ad = expM.topLeftCorner(n, n);
        Bd = expM.topRightCorner(n, m);

        Bdd = Bd * cfg.dt;
    }






vector<vector<double>>MPC::load_data() {
    ifstream file("cornering_stiffness_vs_vertical_load.txt");

    // Controlla se il file è stato aperto correttamente
    if (!file) {
        cerr << "Impossibile aprire il file!" << endl;
        return {};
    }

    vector<vector<double>> numeri(300, vector<double>(2));
    char num1[20], num2[20];

    // Leggi i numeri dal file
    for (int i = 0; i < 300; ++i) {
        if (file >> num1 >> num2) {
            numeri[i][0] = stod(num1);
            numeri[i][1] = stod(num2);
        } else {
            cerr << "Errore nella lettura del file alla riga " << i + 1 << endl;
            break;
        }
    }

    file.close();

    return numeri;
}

// Funzione di interpolazione lineare
double interpolate(double Fx, const vector<vector<double>> &table) {
    // Controllo se Fx è fuori dai limiti della tabella
    if (Fx <= table.front()[0]) {
        return table.front()[1];  // restituisce il primo valore (clamp inferiore)
    }
    if (Fx >= table.back()[0]) {
        return table.back()[1];  // restituisce l’ultimo valore (clamp superiore)
    }

    // Scorro la tabella per trovare l'intervallo corretto
    for (size_t i = 1; i < table.size(); ++i) {
        double F1 = table[i - 1][0];
        double F2 = table[i][0];
        double C1 = table[i - 1][1];
        double C2 = table[i][1];

        if (Fx >= F1 && Fx <= F2) {
            // Interpolazione lineare
            return C1 + (((Fx - F1) / (F2 - F1)) * (C2 - C1));
        }
    }
    // Caso imprevisto (non dovrebbe mai accadere se la tabella è corretta)
    return 0.0;
}



pair<double,double> MPC::load_transfer(double acceleration, const vector<vector<double>> &numeri){
    //calcolo la forza che viene applicata perpendicolarmente sulle ruote
    double Fant = ((m * 9.81 * lb) - (m * acceleration * Ycm)) / la+lb;
    double Frear = ((m * 9.81 * la) + (m * acceleration * Ycm)) / la+lb;

    for (int i = 0; i < 300; ++i) {
        cout << "Riga " << i+1 << " - Primo numero: " << numeri[i][0] << ", Secondo numero: " << numeri[i][1] << endl;
    }

    //ricerca di Ka e Kp (cornering stiffness) con interpolazione
    double Kp = 0;
    double Ka = 0;

    // Interpolazione delle stiffness Ca e Cp
    double Ka = interpolate(Fant, numeri);
    double Kp = interpolate(Frear, numeri);

    pair<double,double> K = make_pair(Ka,Kp);
    return K; //K.first = Ka, K.second = Kp
}
