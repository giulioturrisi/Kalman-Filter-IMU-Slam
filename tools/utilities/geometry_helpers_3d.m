#rotation matrix around x axis
function R=Rx(rot_x)
 c=cos(rot_x);
 s=sin(rot_x);
 R= [1  0  0;
     0  c  -s;
     0  s  c];
end;

#rotation matrix around y axis
function R=Ry(rot_y)
 c=cos(rot_y);
 s=sin(rot_y);
 R= [c  0  s;
     0  1  0;
     -s  0 c];
end;

#rotation matrix around z axis
function R=Rz(rot_z)
 c=cos(rot_z);
 s=sin(rot_z);
 R= [ c  -s  0;
      s  c  0;
      0  0  1];
end;

#derivative of rotation matrix around z
function R=Rx_prime(rot_x)
 dc=-sin(rot_x); #derivative of cos(rot(x))
 ds=cos(rot_x);  #derivative of sin(rot(x))
 R= [0  0  0;
     0  dc  -ds;
     0  ds  dc];
end;

#derivative of rotation matrix around y
function R=Ry_prime(rot_y)
 dc=-sin(rot_y); #derivative of cos(rot(y))
 ds=cos(rot_y);  #derivative of sin(rot(y))
 R= [dc  0 ds;
     0  0  0;
     -ds  0 dc];
end;

#derivative of rotation matrix around z
function R=Rz_prime(rot_z)
 dc=-sin(rot_z); #derivative of cos(rot(z))
 ds=cos(rot_z);  #derivative of sin(rot(z))
 R= [ dc  -ds  0;
      ds  dc  0;
      0  0  0];
end;

#from 6d vector to homogeneous matrix
function T=v2t(v)
    T=eye(4);

    T(1:3,1:3)= euler2Rot(v(4),v(5),v(6));
    T(1:3,4)=v(1:3);
end;

#from homogenous matrix to 6d vector
function v = t2v(T)
    v = zeros(6,1);
    v(1:3) = T(1:3,4);
    R = T(1:3,1:3);
    [phi,theta,psi]  = Rot2euler(R);
    v(4:6) = [phi,theta,psi]';
end;

function S=skew(v)
  S=[0,    -v(3), v(2);
     v(3),  0,    -v(1);
     -v(2), v(1), 0];
end;

function v=flattenIsometry(T)
v=zeros(12,1);
v(1:9)=reshape(T(1:3,1:3)',9,1);
v(10:12)=T(1:3,4);
end;

function T=unflattenIsometry(v)
  T=eye(4);
  T(1:3,1:3)=reshape(v(1:9),3,3)';
  T(1:3,4)=v(10:12);
end;

function v=flattenIsometryByColumns(T)
v=zeros(12,1);
v(1:9)=reshape(T(1:3,1:3),9,1);
v(10:12)=T(1:3,4);
end;

function T=unflattenIsometryByColumns(v)
  T=eye(4);
  T(1:3,1:3)=reshape(v(1:9),3,3);
  T(1:3,4)=v(10:12);
end;

function M=flatTransformationMatrix(v)
  T=unflattenIsometry(v);
  R=T(1:3,1:3);
  t=T(1:3,4);
  M=eye(12);
  M(1:3,1:3)=R';
  M(4:6,4:6)=R';
  M(7:9,7:9)=R';
  M(10,1:3)=t';
  M(11,4:6)=t';
  M(12,7:9)=t';
end;

#derivative of rotation matrix w.r.t rotation around x, in 0
global  Rx0=[0 0 0;
	     0 0 -1;
	     0 1 0];

#derivative of rotation matrix w.r.t rotation around y, in 0
global  Ry0=[0 0 1;
	     0 0 0;
	     -1 0 0];

#derivative of rotation matrix w.r.t rotation around z, in 0
global  Rz0=[0 -1 0;
	     1  0 0;
	     0  0 0];

%Angle normalization between -pi and +pi

function alpha = normalize_angle(alpha)

alpha_temp = mod(alpha,2*pi);

if(abs(alpha_temp) >= pi)
    alpha = -(2*pi - alpha_temp);
  else
    alpha = alpha_temp;
endif
end

%Calculate angular difference in the unit circle
function delta = angdiff(alpha,beta)

delta = alpha - beta;

delta = mod(delta + pi,2*pi) - pi;


end


%Conversion from quaternions representation to Rotation Matrix R = Rz * Ry * Rx

function R = quat2rot(qx,qy,qz,qw)

%Quaternion normalization
  norm = sqrt(qx^2 + qy^2 + qz^2 + qw^2);
  qx = qx / norm;
  qy = qy / norm;
  qz = qz / norm;
  qw = qw / norm;
  
  R = [ qw^2+qx^2-qy^2-qz^2, 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy);
      2*(qx*qy+qw*qz), qw^2-qx^2+qy^2-qz^2, 2*(qy*qz-qw*qx);
      2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), qw^2-qx^2-qy^2+qz^2];
 end


 function [phi,theta,psi] = quat2euler(q_x,q_y,q_z,q_w)

  norm = sqrt(q_x^2 + q_y^2 + q_z^2 + q_w^2);
  q_x = q_x / norm;
  q_y = q_y / norm;
  q_z = q_z / norm;
  q_w = q_w / norm;
  
  #roll (x-axis rotation)

  sinr = 2.0 * (q_w * q_x + q_y * q_z);
  cosr = 1.0 - 2.0 * (q_x * q_x + q_y * q_y);
  phi = atan2(sinr, cosr);

  #pitch (y-axis rotation)
  sinp = 2.0 * (q_w * q_y - q_z * q_x);
 
  if(sinp > 1.0)
    theta=1.0;
  elseif(sinp<-1.0)
    theta=-1.0;
  else
    theta = asin(sinp);
  endif
  #yaw (z-axis rotation)
  siny = 2.0 * (q_w * q_z + q_x * q_y);
  cosy = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);  
  psi = atan2(siny, cosy);

  #phi = normalize_angle(phi);
  #theta = normalize_angle(theta);
  #psi = normalize_angle(psi);

 end

%CONVENCTION R = RZ*RY*RX for euler angles

 function R = euler2Rot(phi,theta,psi)
   
   phi = normalize_angle(phi);
   theta = normalize_angle(theta);
   psi = normalize_angle(psi);

   %Conversion from R = Rx*Ry*Rz
   %R = Rx(phi) * Ry(theta) * Rz(psi);
   %Conversion from R=Rz*Ry*Rx
   R = Rz(psi) * Ry(theta) * Rx(phi);

 end

%Conversion from R to R = Rz*Ry*Rx
function [phi,theta,psi] = Rot2euler(R)

cy = sqrt(R(1,1)^2 + R(2,1)^2);

theta = atan2(-R(3,1),cy);
phi = atan2(R(3,2) / cy, R(3,3) / cy);
psi = atan2(R(2,1)/cy,R(1,1) / cy);

phi = normalize_angle(phi);
theta = normalize_angle(theta);
psi = normalize_angle(psi);

end




