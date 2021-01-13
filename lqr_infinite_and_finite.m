m1 = 1.0;
dt = 0.1;

x0 = [1; 0];

% discrete dynamics
Ad = [1 dt;
     0 1];

Bd = [dt^2/(2*m1);
      dt/m1];
  
% continuous dynamics
A = [0 1;
     0 0];

B = [0;
     1/m1];
 
Q = eye(2);
R = eye(1);
  
%% infinite horizon, discrete, LQ regulator
[Kd,S,e] = lqrd(A,B,Q,R,dt);

%% finite horizon, discrete, LQ regulator

%% simulate
t = 0;
x_hist = [t, x0'];
x_k = x0;

for i=1:1:100
    t = t+dt;
    x_k
    u_k_lqr = -Kd*x_k;
    x_k1 = Ad*x_k + Bd*u_k_lqr;
    x_hist = [x_hist; t, x_k1'];
    x_k = x_k1;

end

plot(x_hist(:,1), x_hist(:,2:3))