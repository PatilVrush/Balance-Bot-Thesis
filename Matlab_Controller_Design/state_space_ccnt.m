mw = 0.03;
mb = 1.0227;
Rw = 0.0401;
Bw = 0.208;
L = 0.1128;
Ir = 0.005;
g = 9.81;
mr = mb;
Igb = 0.000186979406087;
Iw= 2*(Igb+((mw*(Rw^2))/2));
Wnl= 44.1;
Ts = 0.451;

a1 = Iw+((mw+mr)*Rw*Rw);
a2 = mr*Rw*L;
a3 = Ir+(mb*(L*L));
a4 = mb*g*L;
b1 = 2*Ts;
b2 = (2*Ts)/Wnl;

% a1 = 0.0025
% a2 = 0.00099
% a3 = 0.0077
% a4 = 0.164
% b1 = 0.92
% b2 = 0.0212

c1 = (1 - (a2^2/(a1*a3)));
c2 = (1 + (a2/a3));
c3 = (1 + (a2/a1));

TT1=(a4/a3*c1);
TT2= (-b2*c3/a3*c1);
TT3= (-a2*a4/a1*a3*c1);
TT4= (b2*c2/a1*c1);

TT5= (-b1*c3/a3*c1);
TT6 = (b1*c2/a1*c1);

A = [0     1      0      0    ;
     TT1   TT2    0    -TT2   ;
     0      0     0      1    ;
     TT3    TT4   0    -TT4   ];
      
     
 
B = [0;TT5; 0; TT6]; 

C = [1 0  0  0 ;
     0 0  1  0 ];
 
D = [0;0];

states = {'theta' 'theta_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'theta'; 'phi'};

sys_ss = ss(A,B,C,D);
Ts = 1/200;
sys_d = c2d(sys_ss,Ts,'zoh')

co = ctrb(sys_d);
ob = obsv(sys_d);

controllability = rank(co)
observability = rank(ob)

AD = sys_d.a;
BD = sys_d.b;
CD = sys_d.c;
DD = sys_d.d;

Q = C'*C
Q(1,1) = 100;
Q(3,3) = 100
R = 5000;
Nbar = -0.1366;
[K] = dlqr(AD,BD,Q,R)

Ac = [(AD-BD*K)];
Bc = [BD];
Cc = [CD];
Dc = [DD];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac,Bc*Nbar,Cc,Dc,Ts,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.005:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,2),t,y(:,1),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)','fontsize' ,36, 'fontweight','bold')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)','fontsize' ,36, 'fontweight','bold')
title('Step Response with Digital LQR Control')
set(AX,'FontSize',36, 'fontweight','bold' );

