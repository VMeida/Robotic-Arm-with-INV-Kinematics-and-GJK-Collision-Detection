%Bachelor Thesis - UFPE - CTG - Mechanical Engineering
%
%Collision Detection Algorithm for Two Robots
%
%Victor Martins de Almeida
%based on LEITE, A. H. R. algorithm - https://github.com/adrianohrl/6DOF_Manipulator_Robot_Simulation

clc
clear all

%Read STL files

aux = 100;
base = stlread('link0.stl'); base.facecolor = [1 0 0]; 
base.edgecolor = 'none'; base.vertices = base.vertices / aux;
link1 = stlread('link1.stl'); link1.facecolor = [1 .2 .2];
link1.edgecolor = 'none'; link1.vertices = link1.vertices / aux;
link2 = stlread('link2.stl'); link2.facecolor = [1 .4 .4];
link2.edgecolor = 'none'; link2.vertices = link2.vertices / aux;
link3 = stlread('link3.stl'); link3.facecolor = [1 .6 .6];
link3.edgecolor = 'none'; link3.vertices = link3.vertices / aux;
link4 = stlread('link4.stl'); link4.facecolor = [1 .7 .7];
link4.edgecolor = 'none'; link4.vertices = link4.vertices / aux;
link5 = stlread('link5.stl'); link5.facecolor = [1 .95 .95];
link5.edgecolor = 'none'; link5.vertices = link5.vertices / aux;
end_effector = stlread('end_effector.stl'); end_effector.facecolor = [1 .6 .6];
end_effector.edgecolor = 'none'; end_effector.vertices = end_effector.vertices / aux;

link0_2 = stlread('link0_2.stl'); link0_2.facecolor = [0 0 1];
link0_2.edgecolor = 'none'; link0_2.vertices = link0_2.vertices / aux;
link1_2 = stlread('link1_2.stl'); link1_2.facecolor = [.2 .2 1];
link1_2.edgecolor = 'none'; link1_2.vertices = link1_2.vertices / aux;
link2_2 = stlread('link2_2.stl'); link2_2.facecolor = [.4 .4 1];
link2_2.edgecolor = 'none'; link2_2.vertices = link2_2.vertices / aux;
link3_2 = stlread('link3_2.stl'); link3_2.facecolor = [.6 .6 1];
link3_2.edgecolor = 'none'; link3_2.vertices = link3_2.vertices / aux;
link4_2 = stlread('link4_2.stl'); link4_2.facecolor = [.7 .7 1]; 
link4_2.edgecolor = 'none'; link4_2.vertices = link4_2.vertices / aux;
link5_2 = stlread('link5_2.stl'); link5_2.facecolor = [.95 .95 1]; 
link5_2.edgecolor = 'none'; link5_2.vertices = link5_2.vertices / aux;
end_effector_2 = stlread('end_effector_2.stl'); end_effector_2.facecolor = [.6 .6 1];
end_effector_2.edgecolor = 'none'; end_effector_2.vertices = end_effector_2.vertices / aux;

conv = stlread('Conveyor.stl'); conv.facecolor = [.5 .5 .5]; conv.edgecolor = 'none'; conv.vertices = conv.vertices / aux;


%% Declaring links fixed link lengths of the manipulator

global l1 l2 l3 l4 l5 l6 d;

l1 = 445 / aux;
l2 = 150 / aux;
l3 = 700 / aux;
l4 = 115 / aux;
l5 = 795 / aux;
l6 = 85 / aux;
d = 3000 / aux;

iterationsAllowed = 6;

%% Moving parts to their origin and orientation

theta = 0;

[T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics(theta * ones(6, 1));


moveBase2();

link1_cur = transformation(link1, T1_0);
link2_cur = transformation(link2, T2_0);
link3_cur = transformation(link3, T3_0);
link4_cur = transformation(link4, T4_0);
link5_cur = transformation(link5, T5_0);
end_effector_cur = transformation(end_effector, T6_0);

[T0_0 T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics2(theta * ones(6, 1));

link1_2cur = transformation(link1_2, T1_0);
link2_2cur = transformation(link2_2, T2_0);
link3_2cur = transformation(link3_2, T3_0);
link4_2cur = transformation(link4_2, T4_0);
link5_2cur = transformation(link5_2, T5_0);
end_effector_2cur = transformation(end_effector_2, T6_0);


%% Test foward kinematics

%theta_1 = 90;
%theta_2 = 0;
%theta_3 = 0;
%theta_4 = 0;
%theta_5 = 0;
%theta_6 = 0;
%[T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics([theta_1 theta_2 theta_3 theta_4 theta_5 theta_6]);

%link1_cur = transformation(link1, T1_0);
%link2_cur = transformation(link2, T2_0);
%link3_cur = transformation(link3, T3_0);
%link4_cur = transformation(link4, T4_0);
%link5_cur = transformation(link5, T5_0);
%end_effector_cur = transformation(end_effector, T6_0);

%[T0_0 T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics2([theta_1 theta_2 theta_3 theta_4 theta_5 theta_6]);

%link1_2cur = transformation(link1_2, T1_0);
%link2_2cur = transformation(link2_2, T2_0);
%link3_2cur = transformation(link3_2, T3_0);
%link4_2cur = transformation(link4_2, T4_0);
%link5_2cur = transformation(link5_2, T5_0);
%end_effector_2cur = transformation(end_effector_2, T6_0);

%% Plot initial Values
figure, axis equal, view(3), xlabel('x'), ylabel('y'), zlabel('z'), grid on, hold on;
patch(base);
patch(link1_cur);
patch(link2_cur);
patch(link3_cur);
patch(link4_cur);
patch(link5_cur);
patch(end_effector_cur);

patch(link0_2);
patch(link1_2cur);
patch(link2_2cur);
patch(link3_2cur);
patch(link4_2cur);
patch(link5_2cur);
patch(end_effector_2cur);

patch(conv);

%% Set priori angles
global theta_2_priori theta_23_priori theta_3_priori theta_4_priori theta_5_priori theta_6_priori;
theta_2_priori = theta * pi / 180;
theta_23_priori = theta * pi / 180;
theta_3_priori = theta * pi / 180;
theta_4_priori = theta * pi / 180;
theta_5_priori = theta * pi / 180;
theta_6_priori = theta * pi / 180;
%% Define Target 1

T = [1 0 0 1500/aux;0 1 0 0/aux;0 0 -1 400/aux;0 0 0 1];

% Inverse Kinematics
[theta_1 theta_2 theta_3 theta_4 theta_5 theta_6] = inverseKinematics(T);

% Trajectory Planner
qi = [0; 0; 0; 0; 0; 0];
qf = [theta_1; theta_2; theta_3; theta_4; theta_5; theta_6];
q = qf-qi;
tf = 5;
a0 = qi;
a1 = 0;
a2 = (3/(tf^2))*q;
a3 = (-2/(tf^3))*q;

i=1;
t=0;

space=zeros(1,51);
velocity=zeros(1,51);
acceleration=zeros(1,51);
for i=1:51
space(1:6,i)=a0+(a1*t)+(a2*(t.^2))+(a3*(t.^3));
velocity(1:6,i)=a1+(2*a2*t)+(3*a3*(t.^2));
acceleration(1:6,i)=(2*a2)+(6*a3*t);
t=t+0.1;
end

for i = 1:51
%% Refoward kinematics

[T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics(space(1:6,i));
    link1_cur = transformation(link1, T1_0);
    link2_cur = transformation(link2, T2_0);
    link3_cur = transformation(link3, T3_0);
    link4_cur = transformation(link4, T4_0);
    link5_cur = transformation(link5, T5_0);
    end_effector_cur = transformation(end_effector, T6_0);
 
%% Plot refowarded kinematics
  

  cla, hold on;
patch(base);
patch(link1_cur);
patch(link2_cur);
patch(link3_cur);
patch(link4_cur);
patch(link5_cur);
patch(end_effector_cur);

patch(link0_2);
patch(link1_2cur);
patch(link2_2cur);
patch(link3_2cur);
patch(link4_2cur);
patch(link5_2cur);
patch(end_effector_2cur);
patch(conv);

    hold off, axis equal;
    max = l1 + l2 + l3 + l4;
    min = -max;
    axis([-5 35 min max 0 max]);
    view(3), xlabel('x'), ylabel('y'), zlabel('z'), grid on;
    pause(.00001)
    drawnow
end

%Back to position

% Trajectory Planner
qi = qf;
qf = [0; 0; 0; 0; 0; 0];

q = qf-qi;
tf = 5;
a0 = qi;
a1 = 0;
a2 = (3/(tf^2))*q;
a3 = (-2/(tf^3))*q;

i=1;
t=0;

space=zeros(1,51);
velocity=zeros(1,51);
acceleration=zeros(1,51);
for i=1:51
space(1:6,i)=a0+(a1*t)+(a2*(t.^2))+(a3*(t.^3));
velocity(1:6,i)=a1+(2*a2*t)+(3*a3*(t.^2));
acceleration(1:6,i)=(2*a2)+(6*a3*t);
t=t+0.1;
end

%%Simulation
  
for i = 1:51
%% Refoward kinematics

[T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics(space(1:6,i));
    link1_cur = transformation(link1, T1_0);
    link2_cur = transformation(link2, T2_0);
    link3_cur = transformation(link3, T3_0);
    link4_cur = transformation(link4, T4_0);
    link5_cur = transformation(link5, T5_0);
    end_effector_cur = transformation(end_effector, T6_0);
 
%% Plot refowarded kinematics
  

  cla, hold on;
patch(base);
patch(link1_cur);
patch(link2_cur);
patch(link3_cur);
patch(link4_cur);
patch(link5_cur);
patch(end_effector_cur);

patch(link0_2);
patch(link1_2cur);
patch(link2_2cur);
patch(link3_2cur);
patch(link4_2cur);
patch(link5_2cur);
patch(end_effector_2cur);
patch(conv);

    hold off, axis equal;
    max = l1 + l2 + l3 + l4;
    min = -max;
    axis([-5 35 min max 0 max]);
    view(3), xlabel('x'), ylabel('y'), zlabel('z'), grid on;
    pause(.00001)
    drawnow
end

%% Define Target 2

T = [1 0 0 1500/aux;0 1 0 0/aux;0 0 -1 400/aux;0 0 0 1];

% Inverse Kinematics
[theta_1 theta_2 theta_3 theta_4 theta_5 theta_6] = inverseKinematics(T);

% Trajectory Planner
qi = [0; 0; 0; 0; 0; 0];
qf = [theta_1; theta_2; theta_3; theta_4; theta_5; theta_6];
q = qf-qi;
tf = 5;
a0 = qi;
a1 = 0;
a2 = (3/(tf^2))*q;
a3 = (-2/(tf^3))*q;

i=1;
t=0;

space=zeros(1,51);
velocity=zeros(1,51);
acceleration=zeros(1,51);
for i=1:51
space(1:6,i)=a0+(a1*t)+(a2*(t.^2))+(a3*(t.^3));
velocity(1:6,i)=a1+(2*a2*t)+(3*a3*(t.^2));
acceleration(1:6,i)=(2*a2)+(6*a3*t);
t=t+0.1;
end

for i = 1:51
%% Refoward kinematics

[T0_0 T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics2(space(1:6,i));
    link1_2cur = transformation(link1_2, T1_0);
    link2_2cur = transformation(link2_2, T2_0);
    link3_2cur = transformation(link3_2, T3_0);
    link4_2cur = transformation(link4_2, T4_0);
    link5_2cur = transformation(link5_2, T5_0);
    end_effector_2cur = transformation(end_effector_2, T6_0);
 
%% Plot refowarded kinematics
  

  cla, hold on;
patch(base);
patch(link1_cur);
patch(link2_cur);
patch(link3_cur);
patch(link4_cur);
patch(link5_cur);
patch(end_effector_cur);

patch(link0_2);
patch(link1_2cur);
patch(link2_2cur);
patch(link3_2cur);
patch(link4_2cur);
patch(link5_2cur);
patch(end_effector_2cur);
patch(conv);

    hold off, axis equal;
    max = l1 + l2 + l3 + l4;
    min = -max;
    axis([-5 35 min max 0 max]);
    view(3), xlabel('x'), ylabel('y'), zlabel('z'), grid on;
    pause(.00001)
    drawnow
end

%Back to position

% Trajectory Planner
qi = qf;
qf = [0; 0; 0; 0; 0; 0];

q = qf-qi;
tf = 5;
a0 = qi;
a1 = 0;
a2 = (3/(tf^2))*q;
a3 = (-2/(tf^3))*q;

i=1;
t=0;

space=zeros(1,51);
velocity=zeros(1,51);
acceleration=zeros(1,51);
for i=1:51
space(1:6,i)=a0+(a1*t)+(a2*(t.^2))+(a3*(t.^3));
velocity(1:6,i)=a1+(2*a2*t)+(3*a3*(t.^2));
acceleration(1:6,i)=(2*a2)+(6*a3*t);
t=t+0.1;
end

%%Simulation
  
for i = 1:51
%% Refoward kinematics

[T0_0 T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics2(space(1:6,i));
    link1_2cur = transformation(link1_2, T1_0);
    link2_2cur = transformation(link2_2, T2_0);
    link3_2cur = transformation(link3_2, T3_0);
    link4_2cur = transformation(link4_2, T4_0);
    link5_2cur = transformation(link5_2, T5_0);
    end_effector_2cur = transformation(end_effector_2, T6_0);
 
%% Plot refowarded kinematics
  
  cla, hold on;
patch(base);
patch(link1_cur);
patch(link2_cur);
patch(link3_cur);
patch(link4_cur);
patch(link5_cur);
patch(end_effector_cur);

patch(link0_2);
patch(link1_2cur);
patch(link2_2cur);
patch(link3_2cur);
patch(link4_2cur);
patch(link5_2cur);
patch(end_effector_2cur);
patch(conv);

    hold off, axis equal;
    max = l1 + l2 + l3 + l4;
    min = -max;
    axis([-5 35 min max 0 max]);
    view(3), xlabel('x'), ylabel('y'), zlabel('z'), grid on;
    pause(.00001)
    drawnow
end

%% Bad Situation

T = [1 0 0 1500/aux;0 1 0 0/aux;0 0 -1 400/aux;0 0 0 1];

% Inverse Kinematics
[theta_1 theta_2 theta_3 theta_4 theta_5 theta_6] = inverseKinematics(T);

% Trajectory Planner
qi = [0; 0; 0; 0; 0; 0];
qf = [theta_1; theta_2; theta_3; theta_4; theta_5; theta_6];
q = qf-qi;
tf = 5;
a0 = qi;
a1 = 0;
a2 = (3/(tf^2))*q;
a3 = (-2/(tf^3))*q;

i=1;
t=0;

space=zeros(1,51);
velocity=zeros(1,51);
acceleration=zeros(1,51);
for i=1:51
space(1:6,i)=a0+(a1*t)+(a2*(t.^2))+(a3*(t.^3));
velocity(1:6,i)=a1+(2*a2*t)+(3*a3*(t.^2));
acceleration(1:6,i)=(2*a2)+(6*a3*t);
t=t+0.1;
end

for i = 1:51
%% Refoward kinematics

[T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics(space(1:6,i));
    link1_cur = transformation(link1, T1_0);
    link2_cur = transformation(link2, T2_0);
    link3_cur = transformation(link3, T3_0);
    link4_cur = transformation(link4, T4_0);
    link5_cur = transformation(link5, T5_0);
    end_effector_cur = transformation(end_effector, T6_0);

[T0_0 T1_0 T2_0 T3_0 T4_0 T5_0 T6_0] = forwardKinematics2(space(1:6,i));
    link1_2cur = transformation(link1_2, T1_0);
    link2_2cur = transformation(link2_2, T2_0);
    link3_2cur = transformation(link3_2, T3_0);
    link4_2cur = transformation(link4_2, T4_0);
    link5_2cur = transformation(link5_2, T5_0);
    end_effector_2cur = transformation(end_effector_2, T6_0);
 
%% Plot refowarded kinematics
  
  cla, hold on;
patch(base);
patch(link1_cur);
patch(link2_cur);
patch(link3_cur);
robot1 = patch(link4_cur);
patch(link5_cur);
patch(end_effector_cur);

patch(link0_2);
patch(link1_2cur);
patch(link2_2cur);
patch(link3_2cur);
robot2 = patch(link4_2cur);
patch(link5_2cur);
patch(end_effector_2cur);
patch(conv);

    hold off, axis equal;
    max = l1 + l2 + l3 + l4;
    min = -max;
    axis([-5 35 min max 0 max]);
    view(3), xlabel('x'), ylabel('y'), zlabel('z'), grid on;
    pause(.00001)
        collisionTrigger = GJK(robot1,robot2,iterationsAllowed);
    if collisionTrigger
        t = text(3,3,3,'Collision!','FontSize',30);
        break;
    end
    drawnow
    
    
end