mw = 0.03;
mb = 1.0227;
Rw = 0.0401;
Bw = 0.208;
L = 0.1128;
Ir = 0.005;
g = 9.81;
mr = mb;
Igb = 0.000176979406087;
Iw= 2*(Igb+((mw*(Rw^2))/2));
Wnl= 44.1;
Ts = 0.48;

a1 = Iw+((mw+mr)*Rw*Rw);
a2 = mr*Rw*L;
a3 = Ir+(mb*(L*L));
a4 = mb*g*L;
b1 = 2*Ts;
b2 = (2*Ts)/Wnl;

s = tf('s');
G1 = ((-b1)*(a1+a2)*s)/((((-a2^2)+(a1*a3))*s^3)+(b2*(a1+a3+(2*a2))*s^2)-(a1*a4*s)-(a4*b2));
G2 = (((-a2-a3)*s^2)+a4)/((a1+a2)*s^2);

Kp1 = -5;
Ki1 = -27;
Kd1 = -0.36;
C1 = pid(Kp1,Ki1,Kd1);
sys_cl1 = feedback(C1*G1,1);
step(sys_cl1,[0:1:200])
title('PID Control Inner Loop')

Kp2 = 0.023;
Ki2 = 0.005;
Kd2 = 0.002;
C2 = pid(Kp2,Ki2,Kd2);
sys_cl2 = feedback(sys_cl1*G2*C2,1);
step(sys_cl2,[0:1:200])
title('PID Control Outer')
