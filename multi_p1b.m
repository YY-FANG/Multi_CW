clear;

% define the transfer function
num1 = [1 -2]; num2 = [1 1 1.25]; num3 = [1 0.6 9.09];
den1 = [1 -1]; den2 = [1 0.2 1.01]; den3 = [1 0.2 25.01];
num = conv(conv(num1,num2),num3); den = conv(conv(den1,den2),den3);
g_s = tf(num,den);

% derive the state space representation
[ag,bg,cg,dg] = tf2ss(num,den);
sysg = [ag,bg;cg,dg];

w = logspace(-2,3,150);

%% Q12
% w1
Gam = 11;
dnw1 = [1,10,25]; nuw1 = 1;
dnw1i = nuw1; nuw1i = dnw1;
[aw1,bw1,cw1,dw1] = tf2ss(dnw1i*Gam,nuw1i);
sysw1 = [aw1,bw1;cw1,dw1];

% w2
dnw2 = 1; nuw2 = 0.0001;
dnw2i = nuw2; nuw2i = dnw2;
sysw2 = [0.0001];

% new w3
dnw3 = [0.01 1]; nuw3 = [0.01 0];
dnw3i = nuw3; nuw3i = dnw3;
[aw3,bw3,cw3,dw3] = tf2ss(dnw3i,nuw3i);
sysw3 = [aw3,bw3;cw3,dw3];

% P
[rdg,cdg] = size(dg); dim = [5,2,0,1];
[A,B1,B2,C1,C2,D11,D12,D21,D22] = augment(sysg,sysw1,sysw2,sysw3,dim); 
P = ss(A,[B1,B2],[C1;C2],[D11,D12;D21,D22]);

%% Q13
% design H_inf controller
[K,CL,gamma] = hinfsyn(P,1,1); 

%% Q14
% define input variables for pltopt.m
acp = K.A; bcp = K.B; ccp = K.C; dcp = K.D;
acl = CL.A; bcl = CL.B; ccl = CL.C; dcl = CL.D;
