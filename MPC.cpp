#include "MPC.hpp"

MPC::MPC(){
    Q = Q = Eigen::MatrixXd::Zero(nx, nx);
    Q.diagonal() << 10, 10, 5, 2, 2;

    R = Eigen::MatrixXd::Identity(nu, nu)*0.5;
    R_delta = 1;

    //TODO: settare limiti superiori e inferiori di ingressi e stati
    umin = Eigen::VectorXd::Zero(nu);
    umax = Eigen::VectorXd::Zero(nu);
    umin[0] = -24;
    umax[0] = 24;
}


void MPC::updateDiscretization(double vx, double yaw_angle, double acc) {
    A = Eigen::MatrixXd::Zero(nx, nx);
    B = Eigen::MatrixXd::Zero(nx, nu);

    double sin_th = sin(yaw_angle);
    double cos_th = cos(yaw_angle);

    vector<vector<double>> numeri = load_data(); //legge dati da file per le cornering stiffness

    // Prima equazione
    A(0, 2) = -vx * sin_th; // ∂X'/∂θ
    A(0, 3) = -sin_th;   // ∂X'/∂v_y
    
    // Seconda equazione
    A(1, 2) = vx * cos_th;  // ∂Y'/∂θ
    A(1, 3) = cos_th;     // ∂Y'/∂v_y

    // Terza equazione
    A(2, 4) = 1.0;

    // Quarta equazione
    A(3, 5) = -(load_transfer(acc, numeri).first + load_transfer(acc, numeri).second) / (m * vx);
    A(3, 4) = ((load_transfer(acc, numeri).second * lb - load_transfer(acc, numeri).first * la) / (m * vx)) - vx;

    // Quinta equazione
    A(4, 3) = (-la * load_transfer(acc, numeri).first + lb * load_transfer(acc, numeri).second) / (Iz * vx);
    A(4, 4) = -(la * la * load_transfer(acc, numeri).first + lb * lb * load_transfer(acc, numeri).second) / (Iz * vx);

    // Matrice di controllo
    B(3, 0) = load_transfer(acc, numeri).first / m;
    B(4, 0) = (la * load_transfer(acc, numeri).first) / Iz;

    Eigen::MatrixXd M(nx + nu, nx + nu);
    M.setZero();
    M.topLeftCorner(nx, nx) = A * dt;
    M.topRightCorner(nx, nu) = B * dt;
    Eigen::MatrixXd expM = M.exp();

    //matrici discretizzate
    Ad = expM.topLeftCorner(nx, nx);
    Bd = expM.topRightCorner(nx, nu);
}


double MPC::compute(const Eigen::VectorXd& x0, vector<Point> waypoints, double yaw, double velocity){
    // Costruzione pesi
    Eigen::MatrixXd Q_big = Eigen::MatrixXd::Zero(op * nx, op * nx);
    Eigen::MatrixXd R_big = Eigen::MatrixXd::Zero(oc * nu, oc * nu);
    Eigen::MatrixXd Rd_big = Eigen::MatrixXd::Zero((oc - 1) * nu, (oc - 1) * nu);

    for (int i = 0; i < op; ++i) Q_big.block(i * nx, i * nx, nx, nx) = Q;
    for (int i = 0; i < oc; ++i) R_big.block(i * nu, i * nu, nu, nu) = R;
    for (int i = 0; i < oc - 1; ++i) Rd_big.block(i * nu, i * nu, nu, nu) = Rd;

    // Costruzione Sx, Su
    Eigen::MatrixXd Sx = Eigen::MatrixXd::Zero(op * nx, nx);
    Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(op * nx, oc * nu);
    Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(nx, nx);
    for (int i = 0; i < op; ++i) {
        Sx.block(i * nx, 0, nx, nx) = A_power;
        for (int j = 0; j <= i && j < oc; ++j) {
            Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Identity(nx, nx);
            for (int k = 0; k < i - j; ++k)
                A_tmp *= Ad;
            Su.block(i * nx, j * nu, nx, nu) = A_tmp * Bd;
        }
        A_power *= Ad;
    }

    // Costruzione x_ref
    Eigen::VectorXd x_ref_big = Eigen::VectorXd::Zero(op * nx);
    for (int i = 0; i < op; ++i) {
        int idx = std::min(i, static_cast<int>(waypoints.size()) - 1);
        x_ref_big.segment(i * nx, 2) << waypoints[idx].x, waypoints[idx].y;
        // altri stati target (theta, vy, omega) a 0
    }

    // Funzione costo: 1/2 uᵀPu + qᵀu
    Eigen::VectorXd x_pred = Sx * x0;
    Eigen::MatrixXd P = Su.transpose() * Q_big * Su + R_big;
    Eigen::VectorXd q = (x_pred - x_ref_big).transpose() * Q_big * Su;

    // Penalità su Δu = D*u - u_prev
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero((oc - 1) * nu, oc * nu);
    for (int i = 0; i < oc - 1; ++i) {
        D.block(i * nu, i * nu, nu, nu) = -Eigen::MatrixXd::Identity(nu, nu);
        D.block(i * nu, (i + 1) * nu, nu, nu) = Eigen::MatrixXd::Identity(nu, nu);
    }
    P += D.transpose() * Rd_big * D;
    q += (-u_prev.transpose() * Rd_big * D).transpose();

    // Vincoli sullo sterzo
    Eigen::VectorXd lower_bound = Eigen::VectorXd::Constant(oc * nu, umin[0]);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Constant(oc * nu, umax[0]);

    // Setup ottimizzatore OSQP
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(oc * nu);
    solver.data()->setNumberOfConstraints(oc * nu);

    solver.data()->setHessianMatrix(P.sparseView());
    solver.data()->setGradient(q);
    solver.data()->setLinearConstraintsMatrix(Eigen::MatrixXd::Identity(oc * nu, oc * nu).sparseView());
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);

    if (!solver.initSolver()) {
        std::cerr << "OSQP solver init failed\n";
        return 0.0;
    }

    // Warm start
    if (u_prev.size() == oc * nu)
        solver.setPrimalSolution(u_prev);

    if (!solver.solve()) {
        std::cerr << "OSQP solve failed\n";
        return 0.0;
    }

    Eigen::VectorXd u_opt = solver.getSolution();
    u_prev = u_opt;  // salva per warm start prossimo passo
    return u_opt(0); // ritorna solo il primo angolo di sterzo
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
