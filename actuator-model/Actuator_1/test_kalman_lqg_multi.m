% Todorov Neural Comp 2005のモデルに多関節下肢モデルを実装
clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define parameters
global g 
g = 9.8 ; 

prm.dt = 0.01;        % time step (sec)
prm.m = [48 7 4];     % mass (kg) 1: HAT 2: thigh 3: shank 
prm.l = [0 0.5 0.6];  % length (m) 1: dammy 2: thigh 3: shank 
prm.I = [0 prm.m(2)*prm.l(2)^2/12 prm.m(3)*prm.l(3)^2/12]; % inertia 1: dammy 2: thigh 3: shank 
prm.b = [10 10 1000 1000]; % damping (N/sec) b1 b2 bk bg
prm.k = [10000 11000] ; % kk kg

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

%%% initial value
%%% x(t) = [p;pdt;phat] % a: Todorov JNS 2007 を用いて改変
%%% 疑問点：a(加速度次元の筋骨格ダイナミクス）は多関節だと内部で定義できるので外から与える必要ない？？？
X = zeros(1,42) ; % 仮におく

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute system dynamics and cost matrices

dtt = prm.dt/(prm.tau/1000);

%%% Dynamics: x(t+1)  = A x(t) + B (I + Sum(C(i) rnd_1)) u(t) + C0 rnd_n 
A = zeros(42,42); %%% system dynamics matrix (state-dependent) : x(t+1)  = A x(t) + ...
A(1:14,1:14) = eye(14) ; % x
A(1:14,2:15) = eye(14); % x
A(29:42,29:42) = eye(14) ; % xhat, xdtは時間変化するので、kalman_lqg_multiの中に

B = zeros(42,1); %%% system dynamics matrix (control-dependent): x(t+1)  = ... + B (I + Sum(C(i) rnd_1)) u(t) +...
%%% Bも時間変化するので、kalman_lqg_multiの中に

C = prm.c; %%% scaling matrix (control-dependent): x(t+1)  = ... + B (I + Sum(C(i) rnd_1)) u(t) +...
C0 = 0; %%% zero-mean noise: x(t+1)  = ... + C0 rnd_n

%%% Feedback: y(t) = H x(t) + Sum(D(i) rnd_1) x(t) + D0 rnd_n
H = zeros(28,42); %%% Observation matrix; y(t)  = H x(t) + ...
H(1:28,1:28) = eye(28); %%% 決め方は合っているか？？

D = 0; %%% scaling matrix (state-dependent): y(t) = ... + Sum(D(i) rnd_1) x(t) + ...
D0 = diag([prm.pos prm.vel prm.frc]); %%% zero-mean noise: y(t) = ... +  D0 rnd_n

%%% Kalman filter: xhat(t+1) = A xhat(t) + B u(t) + K(t) (y(t) - H xhat(t)) + E0 rnd_n
E0 = 0; %%% zero-mean noise: xhat(t+1) = ... +  E0 rnd_n

%%% Cost per step: cost(t) = u(t)' R u(t) + x(t)' Q(t) x(t)
R = prm.r/N; %%% cost(t) = u(t)' R u(t) + ...
   
Q = zeros(42,42,N); %%% cost(t) = ... + x(t)' Q(t) x(t)
%%% 2005ではForceを状態ベクトルに入れているが、2007や今回はないため、
%%% uの別の項としてPenaltyを与える
%%% しかし最終的にどんな形式がいいのかわからないので、Cost funcは保留
% d = zeros(3,5); % 作り方が不明
% d(1,1) = 1;
% d(1,5) = -1;
% d(2,2) = prm.v;
% d(3,3) = prm.f;
% Q(:,:,N) = d'*d;

%%% Other variables
X1 = zeros(42,1); %%% mean of state paramter: [x;xdt;xhat]
X1(29:42) = T;
S1 = zeros(42,42); %%% covariance matrix of state paramter

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% optimize, plot simulated trajectories

[K,L,Cost,Xa,XSim,CostSim] = ...
   kalman_lqg_multi( A,B,C,C0, H,0,D0, 0, Q,R, X1,S1, 10,1,0,prm,X);
clf;
plot(squeeze(XSim(1,:,:))','r'); hold on;
plot(Xa(1,:),'k','linewidth',2);
xlabel('time step');
ylabel('position');