%% Setup Variables
%clear;
%clc;

syms theta alpha beta gama 'real' %angles
syms dtheta dalpha 'real'
syms ddtheta ddalpha 'real'
syms xcom ycom zcom
syms L1 L2 L3 L4 L5%L1 is 500mm, L2 is 250mm (length of rod), L3 is width and height of rod, L4 is radius of motor
syms m1 m2 m3% m1 mass of composite body, m2 mass of rod, m3 mass of motor
syms g
syms Ix1 Iy1 Iz1
syms F1 F2 f1 f2
syms c1 c2
syms aq1 aq2

q = [theta , alpha]'; % generalised coordinates
dq = [dtheta, dalpha]'; % derivative of generalised coords
ddq = [ddtheta, ddalpha]'; %double derivative of generalised coords
I = diag([Ix1 Iy1 Iz1]);

%% Position vectors

R0_1 = RotZ(alpha);
R1_2 = RotX(theta);
R0_2 = R1_2*R0_1;
R2_0 = transpose(R0_2);

%COM in body frame
r_com_2 = [xcom;ycom;zcom];

%Top of bar in body frame
r_top_base = [0;0;L1];

%COM in inertial frame
r_com_0 = R2_0*r_com_2;

% Position of Forces in body frame
r_f1_2 = [0;L5;0];
r_f2_2 = [0;-L5;0];

%Position Forces in inertial frame
r_f1_0 = R2_0*r_f1_2;
r_f2_0 = R2_0*r_f2_2;
%% Velocity vectors

% Translation veclocity of Composite
dr_com_0 = jacobian(r_com_0, q) * dq;

% Angular Velocity using skew symmetric property
for i = 1:length(R2_0)
    dR2_0(:,i) = jacobian(R2_0(:,i),q)*dq; 
end
dR2_0 = simplify(dR2_0);

sOmega_R2_T_2 = transpose(R2_0)*dR2_0;
sOmega_R2_T_2 = simplify(sOmega_R2_T_2);

omega2_T_2 = [sOmega_R2_T_2(3,2)
              sOmega_R2_T_2(1,3);
              sOmega_R2_T_2(2,1)]; 
omega2_T_2 = simplify(omega2_T_2, 'IgnoreAnalyticConstraints', true);

%% Potential Energy

Vtot = m1*[0 0 g]*r_com_0;

%% Mass moment of inertia

% Moment of inertia of tubes
Ixxr = 3*(1/12 * 0.035 * (0.013^2 + 0.250^2) - 1/12 * .005 * (0.011^2 + .250^2)) + 2*(0.03*.250^2);
Iyyr = 3*(1/12 * 0.035 * (0.013^2 + 0.250^2) - 1/12 * 0.005 * (0.011^2 + 0.250^2));
Izzr = 3*(1/12 * 0.035 * (0.013^2 + 0.013^2) - 1/12 * 0.005 * (0.011^2 + 0.011^2)) + 2*(0.03*.250^2);

% Moment of inertia of motors assumed to be cylinders, taken about pivot
% point
Izzc = 2*(0.5 * 0.058 *  0.014^2 + 0.058*.385^2);
Iyyc = 2*(1/4 * 0.058 * .014^2 + 1/3*0.058*.021^2);
I
% Moment of inertia of speed controller, taken about pivot point
Izzs = 1/6 * 0.030 * (0.025^2 + 0.050^2) + 0.030*.240^2;
Ixxs = 1/6 * 0.030 * (.008^2 + 0.050^2) + 0.030*(0.00105)^2;
Iyys = 1/6 * 0.030 * (0.008^2 + 0.025 ^2) + 0.030*(0.00105)^2;

% Moment of inertia of connector, taken about pivot point
% connector is assumed to be one cylinder and two sqaure tubes
Ixxcon = 0 + 2*0.03*0.125^2;
Iyycon = 0;
Izzcon =  2*0.03*0.125^2;

Ixxpoint = 0.03*0.375^2;
Izzpoint = Ixxpoint;
%4213.01 x mmg 38.4
%2759.88 y mmg 49.145
% 4199.14 z mmg 51


Ix1_ = Ixxr + Ixxs + Ixxcon +Ixxpoint ;
Iy1_ = Iyyr + Iyyc + Iyys + Iyycon;
Iz1_ = Izzr + Izzc + Izzs + Izzcon + Izzpoint;

%% Kinetic Energy

Ttot = 0.5*m1*transpose(dr_com_0)*dr_com_0 + 0.5 * transpose(omega2_T_2) * I *omega2_T_2 ;

       %% Mass Matrix
disp('Mass Matrix')
for i = 1 : length(q)
    
    for j = 1 : length(q)
        
        M(i,j) = diff(diff(Ttot,dq(i)),dq(j));
        
    end
    
end

M = simplify(M);

%% Derivative of Mass Matrix  

dM = sym(zeros(length(M), length(M)));
for i = 1:length(M)
    for j = 1:length(M)
        dM(i,j) = jacobian(M(i,j),q)*dq;
    end
end
dM = simplify(dM);

%% C Matrix -- contains the centrifugal and coriolis accelerations
disp('Coriolis and Centrifugal Matrix')
C = dM * dq - jacobian(Ttot, q)';
C = simplify(C);

%% G Matrix --> Contains the potential energy
disp('Gravity Matrix')
G = sym(zeros(length(M),1));
for i = 1: length(q)
    G(i,1) = diff(Vtot,q(i));
end
G = simplify(G);

%% Forces
%rotation about Y axis to account for changing of angle of force
rot_f_0 = RotY(beta);
rot_0_f = transpose(rot_f_0);

rot_g_0 = RotY(gama);
rot_0_g = transpose(rot_g_0);

F1 = R2_0 * rot_0_f*[0;0;f1]; % thrust force of main rotor
%F2 = R2_0 * rot_0_g*[f2;0;0]; % thrust force of tail rotor
F2 = R2_0 * [f2;0;0];
%% Q matrix --
% generalised force vectors of f1,f2,f3 and f4
Qa = transpose(F1)*diff(r_f1_0, alpha) + transpose(F2)*diff(r_f2_0, alpha); 
Qt = transpose(F1)*diff(r_f1_0, theta) + transpose(F2)*diff(r_f2_0, theta); 

Q = [Qt;Qa];

%% Manipulator
disp('Manipulator')
Manip = simplify(M*ddq + C + G - Q);
P = simplify(M\(Q - C - G));
%P = simplify(M\(Q - transpose(dq)*C - G));
O = Q;
O = subs(O,{beta},{0});
display(Q)


g_ = 9.8;
m1_ = 0.364;
%m2_ = 0.030;
%m3_ = 0.058;
xcom_ = .00335;
ycom_ = .02679;
%ycom_ = 26.79;
zcom_ = 0.00565;
%L1_ = 500;
%L2_ = 250;
%L3_ = 13;
%L4_ = 14;
L5_ = 0.385;

%eqn = subs(eqn, {m1 g beta}, {m1_ g_ 0});
%eqn = subs(eqn, {L5}, {L5_});
%eqn = subs(eqn, {xcom ycom zcom}, {xcom_ ycom_ zcom_});
%eqn = subs(eqn, {Iy1 Iz1}, {Iy1_ Iz1_});

P = subs(P, {m1 g beta gama}, {m1_ g_ 0 0});
P = subs(P, {L5}, {L5_});
P = subs(P, {xcom ycom zcom}, {xcom_ ycom_ zcom_});
P = subs(P, {Ix1 Iy1 Iz1 }, {Ix1_ Iy1_ Iz1_ });

display('Feedback Linearization')
Aq = [aq1;aq2];
Inverse = simplify(O - M*Aq - C*transpose(dq) - G);
Inverse = subs(Inverse, {m1 g}, {m1_ g_});
Inverse = subs(Inverse, {L5}, {L5_});
Inverse = subs(Inverse, {xcom ycom zcom}, {xcom_ ycom_ zcom_});
Inverse = subs(Inverse, {Iy1 Iz1}, {Iy1_ Iz1_});

solF1 = solve(Inverse(1),f1)
solF2 = solve(Inverse(2),f2)


display('Static Linearization')

dyn = [P(1);P(2);dtheta;dalpha];
dyn2 = [P(1);dtheta];

A = jacobian(dyn, [dtheta;dalpha;theta;alpha]);
A = subs(A,{theta alpha dalpha dtheta ddtheta ddalpha f1 f2}, {0, 0,0,0, 0,0,0.248, 0});
A = simplify(A);
A = double(A);



B = jacobian(dyn,[f1; f2]);
B = subs(B,{theta alpha},{0 0});
B = double(B);
c = [0,0,1,0;0,0,0,1];

d = [0,0;0,0];
a = [A,zeros(4,2);-c,zeros(2)]
b = [B;zeros(2)]
c2 = [0,0,1,0,0,0;0,0,0,1,0,0];
d2 = [0,0;0,0];
% b = jacobian(dyn2,[f1]);
% b = subs(b,{theta alpha},{0 0});
% b = double(b)

states = {'dtheta' 'dalpha' 'theta' 'alpha' 'z1' 'z2'};
inputs = {'f1' 'f2'};
outputs = {'theta' 'alpha'};

sys_ss = ss(a,b,c2,d2,'statename',states,'inputname',inputs,'outputname',outputs);
sys_d = c2d(sys_ss,0.1,'zoh');
%a2 = [sys_d.A,zeros(4,2);-c,zeros(2)]
%b2 = [sys_d.B;zeros(2)]
Q = eye(6);
%Q1 = eye(4);
%Q1(3,3) = 300;
%Q(3,3) = 817;
%Q(1,1) = 100;
%Q(2,2) = 0;
%Q(4,4) = 0;
R = eye(2);
K = lqr(a,b,Q,R)
%K1 = dlqr(sys_d.A,sys_d.B,Q,R)
%% Simulink

%new_system('Twin_Rotor8')  % create a simulink model called 'Q2'
%open_system('Twin_Rotor_StateFeedback_PitchandYaw')  % open it
%matlabFunctionBlock('Twin_Rotor_StateFeedback_PitchandYaw/ddtheta_block', P(1))  % add blocks
%matlabFunctionBlock('Twin_Rotor_StateFeedback_PitchandYaw/ddalpha_block', P(2))
%% Simulink Feedback
%matlabFunctionBlock('Twin_Rotor_Inverse_Dynamics_PitchandYaw/Inverse_F1_', solF1);
%matlabFunctionBlock('Twin_Rotor_Inverse_Dynamics_PitchandYaw/Inverse_F2_', solF2);

%% Rotation and Translation
function rot = RotZ(th)
rot = [ cos(th) sin(th) 0 
       -sin(th) cos(th) 0 
              0       0 1  ];
end

function rot = RotX(th)
rot = [ 1        0       0 
        0  cos(th) sin(th) 
        0 -sin(th) cos(th)];
end

function rot = RotY(th)
rot = [ cos(th) 0   -sin(th) 
        0       1       0    
        sin(th) 0   cos(th)];
end

function trans = Trans(x,z)
trans = [1 0 0 x
         0 1 0 0
         0 0 1 z
         0 0 0 1];
end
