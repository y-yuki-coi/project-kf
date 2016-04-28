clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define parameters

prm.dt = 0.01;        % time step (sec)
prm.m = 1;            % mass (kg)
prm.b = 0;            % damping (N/sec)
prm.tau = 40;         % time constant (msec)
prm.c = 0.5;          % control-dependent noise

prm.r = 0.00001;      % control signal penalty
prm.v = 0.2;          % endpoint velocity penalty
prm.f = 0.02;         % endpoint force penalty

prm.pos = 0.5*0.02;   % position noise
prm.vel = 0.5*0.2;    % velocity noise
prm.frc = 0.5*1.0;    % force noise

N = 30;               % duration in number of time steps
T = 0.1;              % target distance


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute system dynamics and cost matrices

dtt = prm.dt/(prm.tau/1000);

%%% Dynamics: x(t+1)  = A x(t) + B (I + Sum(C(i) rnd_1)) u(t) + C0 rnd_n 
A = zeros(5,5); %%% system dynamics matrix (state-dependent) : x(t+1)  = A x(t) + ...
A(1,1) = 1;
A(1,2) = prm.dt;
A(2,2) = 1-prm.dt*prm.b/prm.m;
A(2,3) = prm.dt/prm.m;
A(3,3) = 1-dtt;
A(3,4) = dtt;
A(4,4) = 1-dtt;
A(5,5) = 1;

B = zeros(5,1); %%% system dynamics matrix (control-dependent): x(t+1)  = ... + B (I + Sum(C(i) rnd_1)) u(t) +...
B(4,1) = dtt;

C = prm.c; %%% scaling matrix (control-dependent): x(t+1)  = ... + B (I + Sum(C(i) rnd_1)) u(t) +...
C0 = 0; %%% zero-mean noise: x(t+1)  = ... + C0 rnd_n

%%% Feedback: y(t) = H x(t) + Sum(D(i) rnd_1) x(t) + D0 rnd_n
H = zeros(3,5); %%% Observation matrix; y(t)  = H x(t) + ...
H(1:3,1:3) = eye(3);

D = 0; %%% scaling matrix (state-dependent): y(t) = ... + Sum(D(i) rnd_1) x(t) + ...
D0 = diag([prm.pos prm.vel prm.frc]); %%% zero-mean noise: y(t) = ... +  D0 rnd_n

%%% Kalman filter: xhat(t+1) = A xhat(t) + B u(t) + K(t) (y(t) - H xhat(t)) + E0 rnd_n
E0 = 0; %%% zero-mean noise: xhat(t+1) = ... +  E0 rnd_n

%%% Cost per step: cost(t) = u(t)' R u(t) + x(t)' Q(t) x(t)
R = prm.r/N; %%% cost(t) = u(t)' R u(t) + ...
   
Q = zeros(5,5,N); %%% cost(t) = ... + x(t)' Q(t) x(t)

d = zeros(3,5);
d(1,1) = 1;
d(1,5) = -1;
d(2,2) = prm.v;
d(3,3) = prm.f;
Q(:,:,N) = d'*d;

%%% Other variables
X1 = zeros(5,1); %%% mean of state paramter: [p;p_dt;f;g;p_pr]
X1(5) = T;
S1 = zeros(5,5); %%% covariance matrix of state paramter


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% optimize, plot simulated trajectories

[K,L,Cost,Xa,XSim,CostSim] = ...
   kalman_lqg( A,B,C,C0, H,0,D0, 0, Q,R, X1,S1, 10 );
clf;
plot(squeeze(XSim(1,:,:))','r'); hold on;
plot(Xa(1,:),'k','linewidth',2);
xlabel('time step');
ylabel('position');