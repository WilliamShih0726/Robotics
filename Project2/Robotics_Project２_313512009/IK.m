function joint = IK(NOAP,N)
%inverse kinematics
%% init
%  [   d     a   alpha ]
% DH=[ 00000 00120 -pi/2 
%      00000 00250  0    
%      00000 00260  0 
%      00000 00000 -pi/2 
%      00000 00000  pi/2 
%      00000 00000  0    ];
nx=NOAP(1,1);ny=NOAP(2,1);nz=NOAP(3,1);
ox=NOAP(1,2);oy=NOAP(2,2);oz=NOAP(3,2);
ax=NOAP(1,3);ay=NOAP(2,3);az=NOAP(3,3);
px=NOAP(1,4);py=NOAP(2,4);pz=NOAP(3,4);

a1 = 0.12;
a2 = 0.25;
a3 = 0.26;

joint = [];

% fprintf('////////////////////////////////////////////////////////////////\n\n');
%% solve inverse kinematics
% theta1 ,two solutions--------------------------theta1(1),theta1(2)
theta1_1=atan2(py,px);
theta1_2=atan2(-py,-px);
% theta3 ,four solutions
f1=cos(theta1_1)*px+sin(theta1_1)*py-a1;
f2=cos(theta1_2)*px+sin(theta1_2)*py-a1;
cos31=(f1^2+pz^2-a2^2-a3^2)/(2*a2*a3);
cos32=(f2^2+pz^2-a2^2-a3^2)/(2*a2*a3);
if 1-cos31^2<0
    theta1_1 = 123456;
    theta3_1 = pi/2;
    theta3_2 = pi/2;
else
    theta3_1=atan2(sqrt(1-cos31^2),cos31);
    theta3_2=atan2(-sqrt(1-cos31^2),cos31);
end
if 1-cos32^2<0
    theta1_2 = 123456;
    theta3_3 = pi/2;
    theta3_4 = pi/2;
else
    theta3_3=atan2(sqrt(1-cos32^2),cos32);
    theta3_4=atan2(-sqrt(1-cos32^2),cos32);
end
%theta2
%theta1(1),theta3(1)-------------------> theta23(1),theta2(1)
p1=[f1 -pz
    -pz -f1];
k1=[a3+a2*cos(theta3_1) -a2*sin(theta3_1)]';
r1=p1\k1;
theta23_1=atan2(r1(2,1),r1(1,1));
theta2_1=theta23_1-theta3_1;
%theta1(1),theta3(2)-------------------> theta23(2),theta2(2)
k2=[a3+a2*cos(theta3_2) -a2*sin(theta3_2)]';
r2=p1\k2;
theta23_2=atan2(r2(2,1),r2(1,1));
theta2_2=theta23_2-theta3_2;
%theta1(2),theta3(3)-------------------> theta23(3),theta2(3)
f2=cos(theta1_2)*px+sin(theta1_2)*py-a1;
p2=[f2 -pz
    -pz -f2];
k3=[a3+a2*cos(theta3_3) -a2*sin(theta3_3)]';
r3=p2\k3;
theta23_3=atan2(r3(2,1),r3(1,1));
theta2_3=theta23_3-theta3_3;
%theta1(2),theta3(4)-------------------> theta23(4),theta2(4)
k4=[a3+a2*cos(theta3_4) -a2*sin(theta3_4)]';
r4=p2\k4;
theta23_4=atan2(r4(2,1),r4(1,1));
theta2_4=theta23_4-theta3_4;
%define c23 ,s23
s23_1=r1(2,1);
s23_2=r2(2,1);
c23_1=r1(1,1);
c23_2=r2(1,1);
%theta4
%theta3(1),theta2(1)------------------------------>theta4(1),theta4(2)
g1_1=cos(theta1_1)*c23_1*ax+sin(theta1_1)*c23_1*ay-s23_1*az;
p1_1=-cos(theta1_1)*s23_1*ax-sin(theta1_1)*s23_1*ay-c23_1*az;
theta4_1=atan2(p1_1,g1_1);
theta4_2=atan2(-p1_1,-g1_1);
%theta3(1),theta2(2)------------------------------>theta4(3),theta4(4)
g1_2=cos(theta1_1)*c23_2*ax+sin(theta1_1)*c23_2*ay-s23_2*az;
p1_2=-cos(theta1_1)*s23_2*ax-sin(theta1_1)*s23_2*ay-c23_2*az;
theta4_3=atan2(p1_2,g1_2);
theta4_4=atan2(-p1_2,-g1_2);
%theta5
%theta1(1),theta4(1)------------------------------>theta5(1)
theta5_1=atan2(cos(theta4_1)*g1_1+sin(theta4_1)*p1_1,-sin(theta1_1)*ax+cos(theta1_1)*ay);
%theta1(1),theta4(2)------------------------------>theta5(2)
theta5_2=atan2(cos(theta4_2)*g1_1+sin(theta4_2)*p1_1,-sin(theta1_1)*ax+cos(theta1_1)*ay);
%theta1(1),theta4(3)------------------------------>theta5(3)
theta5_3=atan2(cos(theta4_3)*g1_2+sin(theta4_3)*p1_2,-sin(theta1_1)*ax+cos(theta1_1)*ay);
%theta1(1),theta4(4)------------------------------>theta5(4)
theta5_4=atan2(cos(theta4_4)*g1_2+sin(theta4_4)*p1_2,-sin(theta1_1)*ax+cos(theta1_1)*ay);
%theta6
theta6_1=atan2(-sin(theta1_1)*ox+cos(theta1_1)*oy,sin(theta1_1)*nx-cos(theta1_1)*ny);
theta6_2=atan2(sin(theta1_1)*ox-cos(theta1_1)*oy,-(sin(theta1_1)*nx-cos(theta1_1)*ny));
theta6_3=atan2(-sin(theta1_1)*ox+cos(theta1_1)*oy,sin(theta1_1)*nx-cos(theta1_1)*ny);
theta6_4=atan2(sin(theta1_1)*ox-cos(theta1_1)*oy,-(sin(theta1_1)*nx-cos(theta1_1)*ny));
%definec23,s23
s23_3=r3(2,1);
s23_4=r4(2,1);
c23_3=r3(1,1);
c23_4=r4(1,1);
%theta4
%theta3(3),theta2(3)------------------------------>theta4(5),theta4(6)
g1_3=cos(theta1_2)*c23_3*ax+sin(theta1_2)*c23_3*ay-s23_3*az;
p1_3=-cos(theta1_2)*s23_3*ax-sin(theta1_2)*s23_3*ay-c23_3*az;
theta4_5=atan2(p1_3,g1_3);
theta4_6=atan2(-p1_3,-g1_3);
%theta3(4),theta2(4)------------------------------>theta4(7),theta4(8)
g1_4=cos(theta1_2)*c23_4*ax+sin(theta1_2)*c23_4*ay-s23_4*az;
p1_4=-cos(theta1_2)*s23_4*ax-sin(theta1_2)*s23_4*ay-c23_4*az;
theta4_7=atan2(p1_4,g1_4);
theta4_8=atan2(-p1_4,-g1_4);
%theta5
%theta1(2),theta4(5)------------------------------>theta5(5)
theta5_5=atan2(cos(theta4_5)*g1_3+sin(theta4_5)*p1_3,-sin(theta1_2)*ax+cos(theta1_2)*ay);
%theta1(2),theta4(6)------------------------------>theta5(6)
theta5_6=atan2(cos(theta4_6)*g1_3+sin(theta4_6)*p1_3,-sin(theta1_2)*ax+cos(theta1_2)*ay);
%theta1(2),theta4(7)------------------------------>theta5(7)
theta5_7=atan2(cos(theta4_7)*g1_4+sin(theta4_7)*p1_4,-sin(theta1_2)*ax+cos(theta1_2)*ay);
%theta1(2),theta4(8)------------------------------>theta5(8)
theta5_8=atan2(cos(theta4_8)*g1_4+sin(theta4_8)*p1_4,-sin(theta1_2)*ax+cos(theta1_2)*ay);
%theta6
theta6_5=atan2(-sin(theta1_2)*ox+cos(theta1_2)*oy,sin(theta1_2)*nx-cos(theta1_2)*ny);
theta6_6=atan2(sin(theta1_2)*ox-cos(theta1_2)*oy,-(sin(theta1_2)*nx-cos(theta1_2)*ny));
theta6_7=atan2(-sin(theta1_2)*ox+cos(theta1_2)*oy,sin(theta1_2)*nx-cos(theta1_2)*ny);
theta6_8=atan2(sin(theta1_2)*ox-cos(theta1_2)*oy,-(sin(theta1_2)*nx-cos(theta1_2)*ny));
if theta1_1<=-pi
   theta1_1=theta1_1+2*pi;
end
if theta1_2<=-pi
   theta1_2=theta1_2+2*pi;
end
if theta3_1<=-pi
   theta3_1=theta3_1+2*pi;
end
if theta3_2<=-pi
   theta3_2=theta3_2+2*pi;
end
if theta3_3<=-pi
   theta3_3=theta3_3+2*pi;
end
if theta3_4<=-pi
   theta3_4=theta3_4+2*pi;
end
if theta2_1<=-pi
   theta2_1=theta2_1+2*pi;
end
if theta2_2<=-pi
   theta2_2=theta2_2+2*pi;
end
if theta2_3<=-pi
   theta2_3=theta2_3+2*pi;
end
if theta2_4<=-pi
   theta2_4=theta2_4+2*pi;
end
if theta6_4 == -pi
    theta6_4 = pi;
end
% disp('Inverse Kinematic :')
     
theta=[theta1_1 theta2_1 theta3_1 theta4_1 theta5_1 theta6_1
       theta1_1 theta2_1 theta3_1 theta4_2 theta5_2 theta6_2
       theta1_1 theta2_2 theta3_2 theta4_3 theta5_3 theta6_3
       theta1_1 theta2_2 theta3_2 theta4_4 theta5_4 theta6_4   
       theta1_2 theta2_3 theta3_3 theta4_5 theta5_5 theta6_5      
       theta1_2 theta2_3 theta3_3 theta4_6 theta5_6 theta6_6     
       theta1_2 theta2_4 theta3_4 theta4_7 theta5_7 theta6_7
       theta1_2 theta2_4 theta3_4 theta4_8 theta5_8 theta6_8]*180/pi;

    
%% display
% for i=1:8
%     theta_tmp = theta(i,:);
% %     disp('[ theta1   theta2   theta3   theta4   theta5   theta6]');
%     if theta_tmp(1) > 100
% %         fprintf('out of reach');
%     else
% %         fprintf('% 8.3f ',theta_tmp);
% %         fprintf('\n');
%         if ~isempty(find(exceed_range(theta_tmp), 1))
% %             fprintf('joint ');
% %             fprintf('%d ',find(exceed_range(theta_tmp))');
% %             fprintf('out of range');
%         end
%         joint = theta_tmp;
%     end
% %     fprintf('\n\n');
% end
% if isempty(joint)
%     joint = [0 0 0 0 0 0];
% end
theta_tmp = theta(N,:);
    if theta_tmp(1) > 100
        fprintf('out of reach');
        joint = [0 0 0 0 0 0];
    else
%         fprintf('% 8.3f ',theta_tmp);
%         fprintf('\n');
        if ~isempty(find(exceed_range(theta_tmp), 1))
            fprintf('joint ');
            fprintf('%d ',find(exceed_range(theta_tmp))');
            fprintf('out of range\n');
        end
        joint = theta_tmp;
    end