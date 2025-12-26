function [NOAP, CarPoint] = FK(joint)
% forward kinematics
%% init
theta1 = joint(1)/180*pi;
theta2 = joint(2)/180*pi;
theta3 = joint(3)/180*pi;
theta4 = joint(4)/180*pi;
theta5 = joint(5)/180*pi;
theta6 = joint(6)/180*pi;


DH=[00000 0.120 -pi/2  theta1 
    00000 0.250  0     theta2
    00000 0.260  0     theta3
    00000 00000 -pi/2  theta4
    00000 00000  pi/2  theta5
    00000 00000  0     theta6];

 A = zeros(4,4,6); %n m joint
%% solve forward kinematics
 for i = 1:6
     A(:,:,i)=cell2mat({cos(DH(i,4)) -sin(DH(i,4))*cos(DH(i,3))  sin(DH(i,4))*sin(DH(i,3)) DH(i,2)*cos(DH(i,4));...
                        sin(DH(i,4))  cos(DH(i,4))*cos(DH(i,3)) -cos(DH(i,4))*sin(DH(i,3)) DH(i,2)*sin(DH(i,4));...
                        0 sin(DH(i,3)) cos(DH(i,3)) DH(i,1);...
                        0 0 0 1});
 end
NOAP = A(:,:,1)*A(:,:,2)*A(:,:,3)*A(:,:,4)*A(:,:,5)*A(:,:,6);
X = NOAP(1,4);
Y = NOAP(2,4);
Z = NOAP(3,4);
eul = RM2EA(NOAP(1:3,1:3)); %get euler angle(ZYZ)
CarPoint = [X, Y, Z, eul(1)/pi*180, eul(2)/pi*180, eul(3)/pi*180];
%% display result
% disp('   [     n         o         a         p]');
% disp(NOAP);
% disp('   [     x         y         z       phi     theta       psi]');
% disp(CarPoint);
end