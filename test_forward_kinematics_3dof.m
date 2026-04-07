% Simple runtime test for 3-DOF forward kinematics
disp('Running test_forward_kinematics_3dof...')

tol = 1e-9;
l1 = 1.0; l2 = 1.0; l3 = 1.0;

% Case 1: all zeros -> x = l1+l2+l3, y = 0
[T1, p1] = forward_kinematics_3dof(0, 0, 0, l1, l2, l3);
assert(abs(p1(1) - 3.0) < tol, 'Case 1 failed: x mismatch');
assert(abs(p1(2) - 0.0) < tol, 'Case 1 failed: y mismatch');
assert(abs(T1(1,4) - 3.0) < tol, 'Case 1 failed: T(1,4) mismatch');

% Case 2: first joint at 90 deg, others zero -> x = 0, y = 3
[T2, p2] = forward_kinematics_3dof(pi/2, 0, 0, l1, l2, l3);
assert(abs(p2(1) - 0.0) < tol, 'Case 2 failed: x mismatch');
assert(abs(p2(2) - 3.0) < tol, 'Case 2 failed: y mismatch');
assert(abs(T2(2,4) - 3.0) < tol, 'Case 2 failed: T(2,4) mismatch');

disp('All tests passed.')
disp('Work done: code ran successfully.')
