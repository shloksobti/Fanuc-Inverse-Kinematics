% Lab 2
% fanucFK.m

function [T,fanuc_T] = fanucFK(joint_angles, fanuc)
%{
Input: a 6-element vector of joint angles joint_angles;
       the structure output by the function fanucInit.m
Output: full forward kinematics transform matrix T;
        cell array of tranforms fanuc_T
%}
T01 = dhtf(fanuc.parameters.alpha_0, fanuc.parameters.a_0, fanuc.parameters.d_1, joint_angles(1));
T12 = dhtf(fanuc.parameters.alpha_1, fanuc.parameters.a_1, fanuc.parameters.d_2, pi/2 + joint_angles(2));
T23 = dhtf(fanuc.parameters.alpha_2, fanuc.parameters.a_2, fanuc.parameters.d_3, joint_angles(3));
T34 = dhtf(fanuc.parameters.alpha_3, fanuc.parameters.a_3, fanuc.parameters.d_4, joint_angles(4));
T45 = dhtf(fanuc.parameters.alpha_4, fanuc.parameters.a_4, fanuc.parameters.d_5, joint_angles(5));
T56 = dhtf(fanuc.parameters.alpha_5, fanuc.parameters.a_5, fanuc.parameters.d_6, joint_angles(6));

fanuc_T = {T01 T12 T23 T34 T45 T56};
T = T01 * T12 * T23 * T34 * T45 * T56;

end


