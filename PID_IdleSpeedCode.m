%% By Mohammadreza Maleki
%% a Controller for Engine Idle Speed
%%
close All
clear 
clc
digits(6);
syms dWa Da Dp Dn Df Wa Dd DTl dDp dDn s
Ne=1600; %Nominal Engine Speed

%% Typical parameters of an six-cylinder-engine idle speed

Ka=20;         
Kpm=0.776;
Kn=0.08;
tp=0.21;
Kr=67.2;
tr=3.98;
Hp=13.37;
Hd=10;
Hf=36.6;
T=0.033;

%% State Space & Transfer Function Models

dWa=Ka*Da; % The airflow rate
dDp=(-Dp/tp)+(Kpm*Ka*Da)-(Kpm*Kn*Dn); % The intake manifold pressure
dDn=(-Dn/tr)+Kr*((Hp*Dp)+(Hf*Df)+(Hd*Dd)-(DTl)); % The rotational dynamics model


dX =[dWa;dDp;dDn];
a=[diff(dWa,Wa) diff(dDp,Wa) diff(dDn,Wa) ;diff(dWa,Dp) diff(dDp,Dp)  diff(dDn,Dp); diff(dWa,Dn) diff(dDp,Dn)  diff(dDn,Dn)];
A=vpa(a');
X = [Wa;Dp;Dn];
b1=[diff(dWa,Da) diff(dDp,Da) diff(dDn,Da) ;diff(dWa,Dd) diff(dDp,Dd)  diff(dDn,Dd); diff(dWa,Df) diff(dDp,Df)  diff(dDn,Df)];
B=vpa(b1');
b2=[diff(dWa,DTl) diff(dDp,DTl) diff(dDn,DTl)];
b=vpa(b2');
v=DTl;
C=[0 0 1];
D=[0 0 0];
A1=double(A);
B1=double(B);
C1=double(C);
D1=double(D);
sys=ss(A1,B1,C1,D1);
[NUM,DUM]=ss2tf(A1,B1,C1,D1,3);
Gp=minreal(tf(NUM,DUM)) %Minreal Transfer Function
[NUM,DUM]=tfdata(Gp);
NUM=cell2mat(NUM);
DUM=cell2mat(DUM);

%% Designing a PID COntroller
Kp=0.0102080595074629; % Proportional gain
Ki=0.10944345575242; % Integral gain
Kd=0.000138654213001405; % Derivative gain
c = pid(Kp,Ki,Kd);
G_c=tf(c);
[NUM_c,DUM_c]=tfdata(G_c);
NUM_c=cell2mat(NUM_c);
DUM_c=cell2mat(DUM_c);

%% PID in a Loop with Tf and feedback=1
NUMGs=(conv(NUM_c,NUM));
DUMGs=(conv(DUM_c,DUM));
%Gs=minreal(tf(NUMGs,DUMGs))
Gs=tf(NUMGs,DUMGs)
Gsc=feedback(Gs,1) %feedback = 1

%% Simulation
t = 0:0.0001:1;
U = Ne * ones(size(t));
[Y1,t,X1]=lsim(Gsc,U,t);

%% Figure

figure
plot(t,Y1,'R');
xlabel('Time (seconds)')
ylabel('Engine Idle Speed (RPM)')
xlim ([0 max(t)]) ;
ylim([0 (1.1*Ne)]);
grid on
