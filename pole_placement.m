M = 10;
m = 1;
delta = 1;
b = 1;
L = 0.7;
g = 9.81;

A = [ 0,1,0,0;
      0,-delta/M,m*g/M,0;
      0,0,0,1;
      0,-delta/(M*L),-(m+M)*g/(M*L),0];

B = [0;1;0;1/L];

p = [-100;-100;-100;-100];

K = place(A,B,p)
