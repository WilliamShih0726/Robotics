function RM = EA2RM(EA)
%% ZYZ angle
S1 = sin(EA(1));
C1 = cos(EA(1));
S2 = sin(EA(2));
C2 = cos(EA(2));
S3 = sin(EA(3));
C3 = cos(EA(3));
RM = [C1*C2*C3-S1*S3 -C3*S1-C1*C2*S3 C1*S2;
    C1*S3+C2*C3*S1 C1*C3-C2*S1*S3 S1*S2
    -C3*S2 S2*S3 C2];
end