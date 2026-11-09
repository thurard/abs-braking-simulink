g0 = 9.81;  m = 1000;  v0 = 27.77;  rRoue = 0.3;  J = 0.5;
f_adh = 0.8;
N = m*g0/4;                 % effort normal par roue
                            % (statique, conserve pour les parties 2 et 3)

% Courbe f(g) route sèche (relevée sur la Fig. 1 / le texte)
gliss_tab = [0 0.05 0.10 0.15 0.20 0.30 0.40 0.50 0.60 0.80 1.00];
f_tab     = [0 0.60 0.85 0.93 0.95 0.88 0.80 0.73 0.68 0.63 0.60];

Gliss =0.2;
acc = -1/m;
c_roues = N*rRoue;
A   = 4000;

%% Partie 4 : repartiteur electronique de freinage (EBD)
h    = 0.7;                 % hauteur du centre de gravite (m)
L    = 3;                   % empattement (m)
l_av = 1.5;                 % distance CG - train avant (m)
l_ar = L - l_av;            % distance CG - train arriere (m)
N0   = m*g0/4;              % charge statique par roue (N), reference de l'EBD
tau_a = 0.02;               % constante de temps du transfert de charge (s)
                            % (casse la boucle algebrique a -> N -> F -> a)

% Charges par roue en fonction de l'acceleration a (a < 0 au freinage) :
%   N_av(a) = m*(g0*l_ar - h*a)/(2*L)
%   N_ar(a) = m*(g0*l_av + h*a)/(2*L)
% Coefficients pour les blocs Gain / Constant :
N_av0 = m*g0*l_ar/(2*L);    % charge statique roue avant  (= 2452.5 N)
N_ar0 = m*g0*l_av/(2*L);    % charge statique roue arriere (= 2452.5 N)
k_tr  = m*h/(2*L);          % transfert de charge par roue (N par m/s^2)

% Verifications de la partie 4 (affichees dans la Command Window)
a_test = -5;                % freinage d'urgence (m/s^2)
fprintf('Statique        : train avant %.0f N, train arriere %.0f N\n', ...
        2*N_av0, 2*N_ar0);
fprintf('Freinage a=-5   : train avant %.0f N, train arriere %.0f N\n', ...
        2*(N_av0 - k_tr*a_test), 2*(N_ar0 + k_tr*a_test));
fprintf('Decollage arriere pour |a| = %.1f m/s^2 (max physique ~ %.1f)\n', ...
        g0*l_av/h, max(f_tab)*g0);
