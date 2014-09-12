function [a,b]=rtocarda(R,i,j,k)

%RTOCARDA (Spacelib): Rotation  matrix  to Cardan or Eulerian angles.
% 
% Extracts the Cardan (or Euler) angles from a rotation matrix.
% The  parameters  i, j, k  specify   the   sequence   of  the rotation axes 
% (their value must be the constant (X,Y or Z). 
% j must be different from i and k, k could be equal to i.
% The two solutions are stored in the  three-element vectors q1 and q2.
% RTOCARDA performs the inverse operation than CARDATOR.
% Usage:
%
%			[q1,q2]=rtocarda(R,i,j,k)
%
% Related functions : MTOCARDA 
% 		 
% (c) G.Legnani, C. Moiola 1998; adapted from: G.Legnani and R.Adamini 1993
%___________________________________________________________________________


%spheader
%disp('got this far')
% if ( i<X | i>Z | j<X | j>Z | k<X | k>Z | i==j | j==k )
% 	error('Error in RTOCARDA: Illegal rotation axis ')
% end

if (rem(j-i+3,3)==1)	
    sig=1;   % ciclic 
else
    sig=-1;  % anti ciclic
end
%sig
if (i~=k)  % Cardanic Convention
    %disp('yes!')
	R(j,k);
    R(k,k);
	a(1)= atan2(-sig*R(j,k),R(k,k));
    R(i,k);
	a(2)= asin(sig*R(i,k));
    R(i,j);
    R(i,i);
	a(3)= atan2(-sig*R(i,j),R(i,i));
	
	b(1)= atan2(sig*R(j,k),-R(k,k));
	b(2)= rem( pi-asin(sig*R(i,k)) + pi , 2*pi )-pi;
	b(3)= atan2(sig*R(i,j),-R(i,i));


else % Euleriana Convention

	l=6-i-j;
	
	a(1)= atan2(R(j,i),-sig*R(l,i));
	a(2)= acos(R(i,i));
	a(3)= atan2(R(i,j),sig*R(i,l));

	b(1)= atan2(-R(j,i),sig*R(l,i));
	b(2)= -acos(R(i,i));
	b(3)= atan2(-R(i,j),-sig*R(i,l));

end

% report in degrees instead of radians
a=a*180/pi;
b=b*180/pi;
