% Lab 2
% fanucIK.m

function [is_solution,joint_angles] = fanucIK(T,prev_joint_angles,fanuc)
%{
Input: transform T describing the desired position and orientation;
       6-element vector of the previous joint angles;
       structure output by fanucInit;
Output: boolean variable is_solution;
        6-element vector joint_angles that is the inverse kinematics
        solution nearest to the previous joint angles;
%}
% Parameters
a0 = fanuc.parameters.a_0;
a1 = fanuc.parameters.a_1;
a2 = fanuc.parameters.a_2;
a3 = fanuc.parameters.a_3;
a4 = fanuc.parameters.a_4;
a5 = fanuc.parameters.a_5;

d1 = fanuc.parameters.d_1;
d2 = fanuc.parameters.d_2;
d3 = fanuc.parameters.d_3;
d4 = fanuc.parameters.d_4;
d5 = fanuc.parameters.d_5;
d6 = fanuc.parameters.d_6;

P = T(1:3,4);
Pw = P - T(1:3,1:3)*[0;0;d6];
Pwx = Pw(1);
Pwy = Pw(2);
Pwz = Pw(3);

Pwx1 = sqrt(Pwx^2+Pwy^2);
Pwz1 = Pwz - d1;
Pwx1p = Pwx1 - a1;
Pwy1p = 0;
Pwz1p = Pwz1;


% Check existance of solution
if (P(1) < fanuc.workspace(1)) || (P(1) > fanuc.workspace(2))
    is_solution = false; 
    joint_angles = [];
    return
elseif (P(2) < fanuc.workspace(3)) || (P(2) > fanuc.workspace(4))
    is_solution = false;
    joint_angles = [];
    return
elseif (P(3) < fanuc.workspace(5)) || (P(3) > fanuc.workspace(6))
    is_solution = false;
    joint_angles = [];
    return
else
    is_solution = true;
end

% joint 1
t1 = atan2(Pwy,Pwx);

% joint 3
L25 = sqrt(Pwx1p^2+Pwz1p^2);
sp3 = (L25^2-a2^2-(a3^2+d4^2))/(2*a2*(sqrt(a3^2+d4^2)));
cp3_1 = sqrt(1-sp3^2);
cp3_2 = -1*sqrt(1-sp3^2);

tp3_1 = atan2(sp3,cp3_1);
tp3_2 = atan2(sp3,cp3_1);

t3_1 = tp3_1 - atan2(a3,d4);
t3_2 = tp3_2 - atan2(a3,d4);

% joint 2
ax = a3^2+d4^2;
s2_1 = (-ax*sp3*Pwx1p+(ax*cp3_1-a2)*Pwz1p)/(ax^2-a2^2);
c2_1 = ((ax*cp3_1+a2)*Pwx1p+(ax*sp3)*Pwz1p)/(ax^2-a2^2);
t2_1 = atan2(s2_1,c2_1);

s2_2 = (-ax*sp3*Pwx1p+(ax*cp3_2-a2)*Pwz1p)/(ax^2-a2^2);
c2_2 = ((ax*cp3_2+a2)*Pwx1p+(ax*sp3)*Pwz1p)/(ax^2-a2^2);
t2_2 = atan2(s2_2,c2_2);

M1 = [t1 t2_1 t3_1];
M2 = [t1 t2_2 t3_2];

%%%% Calculating 4,5 6

End_Angles1 = CalculateEndJoints(T, M1);
End_Angles2 = CalculateEndJoints(T, M2);

%%%

M = [M1 End_Angles1(1,:);
      M1 End_Angles1(2,:);
      M2 End_Angles2(1,:);
      M2 End_Angles2(2,:)];


%%% Checking if Joints in Joint Ranges

ValidAngles = CheckJointRanges(M, fanuc);

if isempty(ValidAngles) == true
    is_solution = false;
    joint_angles = [];
    return
end
s = length(ValidAngles(:,1));
N = [];
if s == 1;
    joint_angles = ValidAngles;
    
else
    for i = 1:s
        N(i) = norm([ValidAngles(i,:)] - prev_joint_angles);
    end
    
    [~,I] = min(N);
    joint_angles = [ValidAngles(I,:)];




    
  

end

function End_Angles = CalculateEndJoints(T, Base_Angles)
t1 = Base_Angles(1);
t2 = Base_Angles(2);
t3 = Base_Angles(3);

R_30 = [cos(t1)*cos(t2+t3), sin(t1)*cos(t2+t2), sin(t2+t3);
        -cos(t1)*sin(t2+t3), -sin(t1)*sin(t2+t3), cos(t2+t3);
        sin(t1)              -cos(t1)             0];
R_36 = R_30*T(1:3,1:3);

r13 = R_36(1,3);
r33 = R_36(3,3);
r21 = R_36(2,1);
r23 = R_36(2,3);
r22 = R_36(2,2);

End_Angles = zeros(2,3);

End_Angles(1,1) = atan2(r33,r13);
End_Angles(1,2) = atan2(sqrt(r13^2+r33^2), -r23);
End_Angles(1,3) = atan2(-r22,r21);

End_Angles(2,1) = atan2(-r33,-r13);
End_Angles(2,2) = atan2(-sqrt(r13^2+r33^2), r23);
End_Angles(2,3) = atan2(r22,-r21);

end

function ValidAngles = CheckJointRanges(M, fanuc)

J1 = fanuc.joint_limits{1};
J2 = fanuc.joint_limits{2};
J3 = fanuc.joint_limits{3};
J4 = fanuc.joint_limits{4};
J5 = fanuc.joint_limits{5};
J6 = fanuc.joint_limits{6};

ValidT2 = find(M(:,2) >= J2(1) & M(:,2) < J2(2));
ValidT3 = find(M(:,3) >= J3(1) & M(:,3) < J3(2));
ValidT4 = find(M(:,4) >= J4(1) & M(:,4) < J4(2));
ValidT5 = find(M(:,5) >= J5(1) & M(:,5) < J5(2));
ValidT6 = find(M(:,6) >= J6(1) & M(:,6) < J6(2));

V = intersect(intersect(intersect(intersect(ValidT2,ValidT3),ValidT4),ValidT5),ValidT6);
ValidAngles = M(V,:);
end

end
