%% Compute the B contributions using the partial jacobian formulas
B1 = m1 * (JPL1' * JPL1) + (JOL1'*H0_1(1:3,1:3) * IL1_1 * H0_1(1:3,1:3)'*JOL1);
B2 = m2 * (JPL2' * JPL2) + (JOL2'*H0_2(1:3,1:3) * IL2_2 * H0_2(1:3,1:3)'*JOL2);
B3 = m3 * (JPL3' * JPL3) + (JOL3'*H0_3(1:3,1:3) * IL3_3 * H0_3(1:3,1:3)'*JOL3);

% The final matrix should have all eigenvalues > 0 (positive definite)
B = B1 + B2 + B3;
B = simplify(B);