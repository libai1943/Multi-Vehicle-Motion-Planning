param NE == 10;
var tf  >= 1;
var hi = tf/NE;

param R == 2.5376;
param R1 == 1.5222;
param RL == 1.173;
param UB == 3.75*2.5;
param LB == 3.75*(-1.5);

################################# 车道尺寸参数
set I :={1..NE};
set I1:={1..NE-1};
set J :={1..3};
set K :={0..3};
param tauj{j in K};
param dljtauk{j in K,k in K};
param omega{j in J};



param NC == 12;
set ALL :={1..NC};
param PXPY{j in ALL,k in {1..2}};

################################# 边界限制参数
param amax == 0.5;
param vmax == 15;
param wmax == 0.3;
param phymax == 0.576;

param n == 0.96;
param l = 2.8;
param m = 0.929;
param b = 0.971;

################################# 声明
var x{i in I,j in K,k in ALL};
var y{i in I,j in K,k in ALL};
var x0{i in I,j in K,k in ALL};
var y0{i in I,j in K,k in ALL};
var theta{i in I,j in K,k in ALL};
var v{i in I,j in K,k in ALL};
var phy{i in I,j in K,k in ALL};
var w{i in I,j in K,k in ALL};
var a{i in I,j in K,k in ALL};

################################# 优化时间
var energy{i in I,j in K};
s.t. DIFF_denergydt {i in I,k in J}:
sum{j in K}(dljtauk[j,k]*energy[i,j]) - hi*(sum{jxx in {1..NC}}((phy[i,k,jxx])^2)) = 0;
s.t. EQ_diffenergy {i in I1}:
energy[i+1,0] = sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*energy[i,j]);
s.t. EQ_starting_energy:
energy[1,0] = 0;

minimize criterion:
1;

s.t. tIMEE:
tf = 15;

################################# Vehicle kinematics described via DAEs ####
s.t. EQ_x0x {i in I,j in K,xx in ALL}:
x0[i,j,xx] = x[i,j,xx] + 1.3845;
s.t. EQ_y0y {i in I,j in K,xx in ALL}:
y0[i,j,xx] = y[i,j,xx];

s.t. DIFF_dxdt {i in I, k in J, xx in ALL}:
sum{j in K}(dljtauk[j,k]*x[i,j,xx]) - hi * 10 = 0;

s.t. DIFF_dydt {i in I, k in J, xx in ALL}:
sum{j in K}(dljtauk[j,k]*y[i,j,xx]) = 0;


s.t. EQ_diffx {i in I1, xx in ALL}:
x[i+1,0,xx] = sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*x[i,j,xx]);

s.t. EQ_diffy {i in I1, xx in ALL}:
y[i+1,0,xx] = sum{j in K}((prod{k in K:k<>j}((1-tauj[k])/(tauj[j]-tauj[k])))*y[i,j,xx]);


s.t. Bonds_w {i in I,j in K,xx in ALL}:
w[i,j,xx] == 0;
s.t. Bonds_a {i in I,j in K,xx in ALL}:
a[i,j,xx] == 0;
s.t. Bonds_v {i in I,j in K,xx in ALL}:
v[i,j,xx] == 10;
s.t. Bonds_phy {i in I,j in K,xx in ALL}:
phy[i,j,xx] == 0;

################################# Starting Configurations #################################
s.t. EQ_starting_x_all {xx in ALL}:
x[1,0,xx] = PXPY[xx,1];
s.t. EQ_starting_y_all {xx in ALL}:
y[1,0,xx] = PXPY[xx,2];


s.t. EQ_starting_v_all {xx in ALL}:
v[1,0,xx] = 10;
s.t. EQ_starting_a_all {xx in ALL}:
a[1,0,xx] = 0;
s.t. EQ_ending_a_all {xx in ALL}:
a[NE,3,xx] = 0;
s.t. EQ_ending_w_all {xx in ALL}:
w[NE,3,xx] = 0;
s.t. EQ_ending_theta_all {xx in ALL}:
theta[NE,3,xx] = 0;




data;
param: PXPY := include PXPY;
	
param: dljtauk :=
         0         0   -9.0000
         0    1.0000   -4.1394
         0    2.0000    1.7394
         0    3.0000   -3.0000
    1.0000         0   10.0488
    1.0000    1.0000    3.2247
    1.0000    2.0000   -3.5678
    1.0000    3.0000    5.5320
    2.0000         0   -1.3821
    2.0000    1.0000    1.1678
    2.0000    2.0000    0.7753
    2.0000    3.0000   -7.5320
    3.0000         0    0.3333
    3.0000    1.0000   -0.2532
    3.0000    2.0000    1.0532
    3.0000    3.0000    5.0000;

param: tauj :=
	0		0
	1		0.1550510257216822
	2		0.6449489742783178
	3		1.0;

param: omega:=
	1		 3.76403062700467e-1 
	2		 5.12485826188421e-1
	3		 1.11111111111111e-1;