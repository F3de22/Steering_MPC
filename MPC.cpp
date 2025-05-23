#include "MPC.hpp"

MPC::MPC() : solver_initialized(false){
    Q = Eigen::MatrixXd::Zero(nx, nx);
    Q.diagonal() << 10, 10, 0, 0, 0;

    R = Eigen::MatrixXd::Identity(nu, nu)*0.5;
    R_delta = Eigen::MatrixXd::Identity(1, 1); //moltiplicare per un valore se si vuole cambiare peso

    //TODO: settare limiti superiori e inferiori di ingressi e stati
    umin = Eigen::VectorXd::Zero(nu);
    umax = Eigen::VectorXd::Zero(nu);
    umin[0] = -24* 3.14 / 180.0;
    umax[0] = 24* 3.14 / 180.0;
    xmin = Eigen::VectorXd::Zero(nx);
    xmax = Eigen::VectorXd::Zero(nx);
    xmin[0] = 1.5;
    xmax[0] = 1.5;

    u_prev = Eigen::VectorXd::Zero(oc * nu);
    numeri = load_data();//legge dati da file per le cornering stiffness
}

void MPC::updateDiscretization(double vx, double yaw_angle, double acc) {
    A = Eigen::MatrixXd::Zero(nx, nx);
    B = Eigen::MatrixXd::Zero(nx, nu);

    double sin_th = sin(yaw_angle);
    double cos_th = cos(yaw_angle);

    std::pair<double, double> K_values = load_transfer(acc, numeri);
    double Ka = K_values.first;  // Stiffness anteriore
    double Kp = K_values.second; // Stiffness posteriore

    // Prima equazione
    A(0, 2) = -vx * sin_th; // ∂X'/∂θ
    A(0, 3) = -sin_th;   // ∂X'/∂v_y
    
    // Seconda equazione
    A(1, 2) = vx * cos_th;  // ∂Y'/∂θ
    A(1, 3) = cos_th;     // ∂Y'/∂v_y

    // Terza equazione
    A(2, 4) = 1.0;

    // Quarta equazione
    A(3, 5) = -(Kp + Ka) / (m * vx);
    A(3, 4) = ((Kp * lb - Ka * la) / (m * vx)) - vx;

    // Quinta equazione
    A(4, 3) = (-la * Ka + lb * Kp) / (Iz * vx);
    A(4, 4) = -(la * la * Ka + lb * lb * Kp) / (Iz * vx);

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
    // Costruzione matrici di peso a blocchi
    Eigen::MatrixXd Q_blk = Eigen::MatrixXd::Zero(op * nx, op * nx);
    Eigen::MatrixXd R_blk = Eigen::MatrixXd::Zero(oc * nu, oc * nu);
    Eigen::MatrixXd Rd_blk = Eigen::MatrixXd::Zero((oc - 1) * nu, (oc - 1) * nu);

    for (int i = 0; i < op; ++i) 
        Q_blk.block(i * nx, i * nx, nx, nx) = Q; // crea una matrice a blocchi diagonale (con la matrice Q su ogni blocco della diagonale)
    for (int i = 0; i < oc; ++i) 
        R_blk.block(i * nu, i * nu, nu, nu) = R;
    for (int i = 0; i < oc - 1; ++i) 
        Rd_blk.block(i * nu, i * nu, nu, nu) = R_delta;

    // Costruzione delle matrici predittive Sx, Su
    Eigen::MatrixXd Sx = Eigen::MatrixXd::Zero(op * nx, nx); // Sx: Predizione dello stato futuro basata sullo stato iniziale (x0)
    Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(op * nx, oc * nu); // Su: Predizione dello stato futuro basata sulla sequenza di input (u)

    Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(nx, nx); // Ad^0 = I
    for (int i = 0; i < op; ++i) {
        Sx.block(i * nx, 0, nx, nx) = A_power; // Sx_i = Ad^i
        for (int j = 0; j <= i && j < oc; ++j) {
            Eigen::MatrixXd A_tmp = Eigen::MatrixXd::Identity(nx, nx);
            for (int k = 0; k < i - j; ++k)
                A_tmp *= Ad;
            Su.block(i * nx, j * nu, nx, nu) = A_tmp * Bd;
        }
        A_power *= Ad;
    }

    // Costruzione x_ref_big
    Eigen::VectorXd x_ref_big = Eigen::VectorXd::Zero(op * nx);
    for (int i = 0; i < op; ++i) {
        int idx = std::min(i, static_cast<int>(waypoints.size()) - 1);
        x_ref_big.segment(i * nx, 2) << waypoints[idx].x, waypoints[idx].y;
        // altri stati (theta, vy, omega) si lasciano a 0
    }

    // Funzione costo: 1/2 uᵀPu + qᵀu
    Eigen::MatrixXd P = Su.transpose() * Q_blk * Su + R_blk;
    Eigen::VectorXd x_pred = Sx * x0; // Predizione dello stato senza input (solo con x0)
    Eigen::VectorXd q = (x_pred - x_ref_big).transpose() * Q_blk * Su;

    // Penalità su Δu = D*u - u_prev
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero((oc - 1) * nu, oc * nu);
    for (int i = 0; i < oc - 1; ++i) {
        // D contiene le differenze tra due ingressi successivi [uk - uk-1]
        D.block(i * nu, i * nu, nu, nu) = -Eigen::MatrixXd::Identity(nu, nu); // -u_{k-1}// D contiene le differenze tra due ingressi successivi [uk - uk-1]
        D.block(i * nu, (i + 1) * nu, nu, nu) = Eigen::MatrixXd::Identity(nu, nu); // +u_k
    }
    P += D.transpose() * Rd_blk * D;
    q += (-u_prev.transpose() * Rd_blk * D).transpose();

    // Vincoli sullo sterzo
    Eigen::VectorXd lower_bound = Eigen::VectorXd::Constant(oc * nu, umin[0]);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Constant(oc * nu, umax[0]);

    // Setup ottimizzatore OSQP che calcola il controllo ottimo
    if (!solver_initialized) {
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);

    solver.data()->setNumberOfVariables(oc * nu);
    solver.data()->setNumberOfConstraints(oc * nu);
    Eigen::SparseMatrix<double> P_s = P.sparseView();
    if (!solver.data()->setHessianMatrix(P_s)) return 0.0;
    if (!solver.data()->setGradient(q)) return 0.0;
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(oc * nu, oc * nu);
    Eigen::SparseMatrix<double> A_s = A.sparseView();
    if (!solver.data()->setLinearConstraintsMatrix(A_s)) return 0.0;
    if (!solver.data()->setLowerBound(lower_bound)) return 0.0;
    if (!solver.data()->setUpperBound(upper_bound)) return 0.0;

    if (!solver.initSolver()) {
        std::cerr << "OSQP solver init failed\n";
        return 0.0;
    }

    solver_initialized = true;
    } else {
        // aggiornamento dei dati per il warm start
        Eigen::SparseMatrix<double> P_s = P.sparseView();
        if (!solver.updateHessianMatrix(P_s)) return 0.0;
        if (!solver.updateGradient(q)) return 0.0;
        if (!solver.updateBounds(lower_bound, upper_bound)) return 0.0;
    }

    if (!solver.solve()) {
        std::cerr << "OSQP: Risoluzione fallita\n";
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

    // Leggi i numeri dal file
    vector<vector<double>> numeri;
    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        double val1, val2;
        if (ss >> val1 >> val2) {
            numeri.push_back({val1, val2});
        } else {
            cerr << "Errore nella lettura della riga: " << line <<endl;
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
    double Fant = ((m * 9.81 * lb) - (m * acceleration * Ycm)) / (la+lb);
    double Frear = ((m * 9.81 * la) + (m * acceleration * Ycm)) / (la+lb);

    // Interpolazione delle stiffness Ka e Kp
    double Ka = interpolate(Fant, numeri);
    double Kp = interpolate(Frear, numeri);

    pair<double,double> K = make_pair(Ka,Kp);
    return K; //K.first = Ka, K.second = Kp
}
