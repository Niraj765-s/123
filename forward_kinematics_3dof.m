function [T, position] = forward_kinematics_3dof(theta1, theta2, theta3, l1, l2, l3)
%FORWARD_KINEMATICS_3DOF Simple 3-DOF planar robot forward kinematics.
%   Inputs:
%       theta1, theta2, theta3 : joint angles in radians
%       l1, l2, l3             : link lengths
%   Outputs:
%       T        : 4x4 homogeneous transform of end-effector wrt base
%       position : [x; y; z] end-effector position

    t12 = theta1 + theta2;
    t123 = t12 + theta3;

    x = l1*cos(theta1) + l2*cos(t12) + l3*cos(t123);
    y = l1*sin(theta1) + l2*sin(t12) + l3*sin(t123);
    z = 0;

    R = [cos(t123), -sin(t123), 0;
         sin(t123),  cos(t123), 0;
         0,          0,         1];

    T = [R, [x; y; z];
         0, 0, 0, 1];

    position = [x; y; z];
end
