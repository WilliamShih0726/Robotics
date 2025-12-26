function EA = RM2EA(RM)
%% ZYZ angle
EA(1) = atan2(RM(2,3), RM(1,3));
EA(2) = atan2(cos(EA(1))*RM(1,3)+sin(EA(1))*RM(2,3), RM(3,3));
EA(3) = atan2(-sin(EA(1))*RM(1,1)+cos(EA(1))*RM(2,1), -sin(EA(1))*RM(1,2)+cos(EA(1))*RM(2,2));
end