clear;

%% the model of the HIMAT aircraft
ag =[
-2.2567e-02  -3.6617e+01  -1.8897e+01  -3.2090e+01   3.2509e+00  -7.6257e-01;
9.2572e-05  -1.8997e+00   9.8312e-01  -7.2562e-04  -1.7080e-01  -4.9652e-03;
1.2338e-02   1.1720e+01  -2.6316e+00   8.7582e-04  -3.1604e+01   2.2396e+01;
0            0   1.0000e+00            0            0            0;
0            0            0            0  -3.0000e+01            0;
0            0            0            0            0  -3.0000e+01];

bg = [0     0;
      0     0;
      0     0;
      0     0;
     30     0;
      0    30];
  
cg = [0     1     0     0     0     0;
      0     0     0     1     0     0];
  
dg = [0     0;
      0     0];
  
ss_g = ss(ag,bg,cg,dg);
  
%% preliminary analysis
% computing the poles of the plant
poleg  = eig(ag); 

% computing the transmission zeros
tzerog = tzero(ag,bg,cg,dg);

% computing SV Bode plot of the open loop plant
w = logspace(-3,5,50);
svg = sigma(ag,bg,cg,dg,1,w); svg = 20*log10(svg);

figure(1);
semilogx(w,svg)
title('MIMO Fighter Pitch Axis Open Loop')
xlabel('Frequency - Rad/Sec'); ylabel('SV - db'); grid on

%% Q1
% w1
Gam = 16.8; % step (3) (design 1)
% Gam = 8.4; % step (4) (design 2)
dnw1 = [1.0000e+00 1.0000e-02]; nuw1 = [0 1];
dnw1i = nuw1; nuw1i = dnw1;
w1 = [Gam*dnw1i;nuw1i;Gam*dnw1i;nuw1i];
svw1i = bode(nuw1i,dnw1i,w); svw1i = 20*log10(svw1i);

%w2
dnw2 = 1; nuw2 = 0.0001; % step (3) (design 1)
% dnw2 = [1,1.0000e+04]; nuw2 = [1,1]; % step (4) (design 2)
dnw2i = nuw2; nuw2i = dnw2;
w2 = [dnw2i;nuw2i;dnw2i;nuw2i];
svw2i = bode(nuw2i,dnw2i,w); svw2i = 20*log10(svw2i);

% w3
k = 1000; tau = 5.0000e-04; 
dnw3 = [0 0 k]; nuw3 = [1 0 0];
dnw3i = nuw3; nuw3i = dnw3;
w3 = [0 1 0 0;0 0 0 k;tau 1 0 0;0 0 0 k];
svw3i = bode(nuw3i,dnw3i,w); svw3i = 20*log10(svw3i);

% plot of the weightings
figure(2)
semilogx(w,svw1i,w,svw3i)
grid on
title('MIMO Fighter Design -- Design Specifications')
xlabel('Frequency - Rad/Sec')
ylabel('1/W1 & 1/W3 - db')
text(0.003,-70,'Sensitivity Spec.')
text(0.003,-100,'1/W1(s)')
text(200,-20,'Robustness Spec.')
text(1000,-50,'1/W3(s)')
legend('1/W1(s)','1/W3(s)')

%% Q2
% P
P = augtf(ss_g,w1,w2,w3); 

% design H_inf controller
[ss_cp,ss_cl] = hinfsyn(P,2,2);

% define input variables for pltopt.m
[acp,bcp,ccp,dcp] = ssdata(ss_cp); [acl,bcl,ccl,dcl] = ssdata(ss_cl);

%% Q4
% computing Bode plots of sensitivity & complementary sensitivity
[al,bl,cl,dl] = series(acp,bcp,ccp,dcp,ag,bg,cg,dg);
[als,bls,cls,dls] = feedbk(al,bl,cl,dl,1);
sysk = ss(acp,bcp,ccp,dcp); syss = ss(als,bls,cls,dls);
sysks = series(sysk,syss);
svk = sigma(sysks,w);  svk = 20*log10(svk);
figure(3);
semilogx(w,svk,w,svw2i);
title('Control Effort and 1/W2');
xlabel('Frequency - Rad/Sec');
ylabel('Gain - db');
grid on
