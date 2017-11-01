function [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6] = inverseKinematics(T_given)

global l1 l2 l3 l4 l5 l6 theta_2_priori theta_23_priori theta_3_priori theta_4_priori theta_5_priori theta_6_priori;
% PART 1: (Wrist Position)

%         [x']   [x]               [r13]
%   Pw =  [y'] = [y] - (l6) *      [r23]
%         [z']   [z]               [r33]

x = T_given(1, 4) - (l6) * T_given(1, 3);
y = T_given(2, 4) - (l6) * T_given(2, 3);
z = T_given(3, 4) - (l6) * T_given(3, 3);
xc = sqrt(x^2+y^2);

%% Arm
% Joint 1
theta_1 = atan2(y,x);

%Joint 2

% a*cos(theta) - b*sin(theta) = K

a = -2*l5*(xc-l2)-2*l4*(z-l1);
b = -2*l4*(xc-l2)+2*l5*(z-l1);
c = l3^2 - l4^2 - l5^2 - (xc-l2)^2 - (z-l1)^2;

theta = tanHalfAngleIdentity(b,a,c);

if abs(abs(theta(1)) - abs(theta_23_priori)) == abs(abs(theta(2)) - abs(theta_23_priori))
    if abs(theta(1) - theta_23_priori) < abs(theta(2) - theta_23_priori)
        theta_23 = theta(1);
    else
        theta_23 = theta(2);
    end;
elseif abs(abs(theta(1)) - abs(theta_23_priori)) < abs(abs(theta(2)) - abs(theta_23_priori))
    theta_23 = theta(1);
else
    theta_23 = theta(2);
end;
theta_23_priori = theta_23;

K1 = xc - l2 - l4*sin(theta_23) - l5*cos(theta_23);
K2 = z - l1 - l4*cos(theta_23) + l5*sin(theta_23);

theta_2 = atan2(K1,K2);
%theta_3 = theta_23 - theta_2;

% Joint 3
%    a * cos(theta_23) - b * sin(theta_23) = K

theta_3 = theta_23 - theta_2 ;


%% Wrist

R03= [ cos(theta_1)*cos(theta_2)*sin(theta_3) + cos(theta_1)*cos(theta_3)*sin(theta_2), sin(theta_1), cos(theta_1)*cos(theta_2)*cos(theta_3) - cos(theta_1)*sin(theta_2)*sin(theta_3);
    cos(theta_2)*sin(theta_1)*sin(theta_3) + cos(theta_3)*sin(theta_1)*sin(theta_2), -cos(theta_1),cos(theta_2)*cos(theta_3)*sin(theta_1) - sin(theta_1)*sin(theta_2)*sin(theta_3);
                cos(theta_2)*cos(theta_3) - sin(theta_2)*sin(theta_3),        0, - cos(theta_2)*sin(theta_3) - cos(theta_3)*sin(theta_2)];
            
R06 = T_given(1:3,1:3);

R46 = R03/R06;

theta = atan2(R46(2,3),R46(1,3))*[1 -1];
if abs(abs(theta(1)) - abs(theta_4_priori)) == abs(abs(theta(2)) - abs(theta_4_priori))
    if abs(theta(1) - theta_4_priori) < abs(theta(2) - theta_4_priori)
        theta_4 = theta(1);
    else
        theta_4 = theta(2);
    end;
elseif abs(abs(theta(1)) - abs(theta_4_priori)) < abs(abs(theta(2)) - abs(theta_4_priori))
    theta_4 = theta(1);
else
    theta_4 = theta(2);
end;
theta_4_priori = theta_4;


theta_5 = atan2(-(sin(theta_4)*R46(2,3)+cos(theta_4)*R46(1,3)),R46(3,3));

theta = atan2(-R46(3,2),R46(3,1))*[1 -1];
if abs(abs(theta(1)) - abs(theta_6_priori)) == abs(abs(theta(2)) - abs(theta_6_priori))
    if abs(theta(1) - theta_6_priori) < abs(theta(2) - theta_6_priori)
        theta_6 = theta(1);
    else
        theta_6= theta(2);
    end;
elseif abs(abs(theta(1)) - abs(theta_6_priori)) < abs(abs(theta(2)) - abs(theta_6_priori))
    theta_6 = theta(1);
else
    theta_6 = theta(2);
end;
theta_6_priori = theta_6;


%% converting to degrees
theta_1 = theta_1 * 180 / pi;
theta_2 = theta_2 * 180 / pi;
theta_23 = theta_23 * 180 / pi;
theta_3 = theta_3 * 180 / pi;
theta_4 = theta_4 * 180 / pi;
theta_5 = theta_5 * 180 / pi;
theta_6 = theta_6 * 180 / pi;