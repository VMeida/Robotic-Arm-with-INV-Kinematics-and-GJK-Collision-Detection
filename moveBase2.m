%% Link 0
% Moving base to align with frame 0
T = [-1    0    0   3000 / aux
    0    1    0   0 / aux
    0    0    1    0
    0    0    0    1];
link0_2 = transformation(link0_2, T);

T = [1    0    0   1500 / aux
    0    1    0   0 / aux
    0    0    1    0
    0    0    0    1];
conv = transformation(conv, T);

%% Ploting each part at the origin:

 %figure, axis equal, view(3), xlabel('x'), ylabel('y'), zlabel('z'), grid on, hold on;
 %patch(link0_1_new);
 %patch(link1_1_new);
 %patch(link2_1_new);
 %patch(link3_1_new);
 %patch(link4_1_new);
 %patch(link5_1_new);
 %patch(end_effector_1_new);
 
 %patch(link0_2_new);
 %patch(link1_2_new);
 %patch(link2_2_new);
 %patch(link3_2_new);
 %patch(link4_2_new);
 %patch(link5_2_new);
 %patch(end_effector_2_new);