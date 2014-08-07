% This program is called from the shell program
% "run_upper_extremity_kinematics.m"
% That particular program reads in a motion capture file and saves it as an
% array called "mean_pos".  
%
% This file assumes that the following data (in this order) is included in
% the input file:
% Frame#, Time, xyphoid process (ribcage), T8 (posterior), 
% sternal notch (ribcage), C7 (posterior), Acromion process, 
% Glenohumeral cntr of rot. (post), Medial epicondyle, lateral epicondyle, 
% ulnar styloid, radial styloid, 2nd metacarpal head		

% We only marked subjects' dominant arms.  For now assume this is always
% the right arm
%For each marker there are 3 columns with position in X,Y,Z 
%
% There can be other columns after this dataset, however they will be
% ignored for now.
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  First we need to read the static files to get a reference for what is 0
%%  and 90 degrees.  The first static trial has the shoulder at 0 flexion,
%%  0 abduction, elbow at 90 deg flexion, wrist neutral and pronated.

% Reminder of marker order:
% 1 Frame 
% 2 Time
% 3-5 xyphoid process
% 6-8 T8   
% 9-11 sternal notch   
% 12-14 C7  
% 15-17 Acromion process
% 18-20 Glenohumeral cntr of rot. (post)
% 21-23 Medial epicondyle
% 24-26 lateral epicondyle
% 27-29 ulnar styloid
% 30-32 radial styloid
% 33-35 2nd metacarpal head	
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define Anatomic Coordinate System based on ISG recomendations and U
% Deleware data.  Start with trunk coordinate system
%
% Trunk origin at midpoint of T8 and Xyphoid process
%�	Y-vector from origin to midpoint of C7 and sternal notch 
%�	X-vector is Y-vector crossed onto vector from xyphoid to T8 (AP)
%�	Z-vector from X crossed onto Y (ML)
%
%  Note that in our data captures, relative to the global reference frame,
%  subjects were facing in the -X direction and in gloabal, Z is up.

% the array "mean_pos" is the array that contains all of the data
time=mean_pos(2);
xyphoid_T8=mean_pos(3:5)-mean_pos(6:8); % vector from T8 to xyphoid (pointing anterior)
trunk_center=mean_pos(3:5)-0.5*xyphoid_T8;  % midpoint from xyphoid to T8/ trunk origin
C7_sternal=mean_pos(9:11)-mean_pos(12:14); % vector from C7 to sternal notch (pointing anterior)
c7s_midpt=mean_pos(12:14)+0.5*C7_sternal;  % midpoint of C7/sternal (directly UP from trunk origin)
trunkY=c7s_midpt-trunk_center; %vector pointing from inferior to superior within the trunk (UP)
trunkZ=cross(trunkY,xyphoid_T8);  %vector pointing LEFT.  The sign needs to be flipped to fix this (see line 86)
trunkX=cross(trunkY,trunkZ);  %vector pointing POSTERIOR.  The sign needs to be flipped (see line 85)
% Now make all into unit vectors.  Use the function mag3, which
% returns the magnitude of the 3d vector.
trunkY=trunkY/mag3(trunkY);  %mag3 just finds the magnitude of the vector
trunkX=-trunkX/mag3(trunkX); %correct the sign so X points anterior
trunkZ=-trunkZ/mag3(trunkZ); %correct the sign so Z points right
trunkE=[trunkX;trunkY;trunkZ];
trunk_origin=mean_pos(6:8); %ISB convention is origin at sternal notch

% Scapula (Shoulder)
%�	X-vector from midpoint of C7 and Suprasternal Notch to the Acromion
%   Process marker (-10 mm in global z direction to account for marker
%   height) (AP)
%�	Z-vector from shoulder X-vector crossed onto trunk Y-vector (ML,
%pointing along elbow axis of rotation, right)
%�	Y-vector from shoulder Z-vector crossed onto shoulder X-vector (Axial)
fixedz=mean_pos(17)-10;  %see note on line 91
acromion=[mean_pos(15:16),fixedz];  %this is the shoulder origin
% NOTE: currently this simply translates down in global z direction.
% Ideally, we would instead translate 10 mm in the negative trunkY
% direction
shoulderX=acromion-c7s_midpt;
shoulderZ=cross(shoulderX,trunkY);
shoulderY=cross(shoulderZ,shoulderX);
% normalize into unit vectors
shoulderY=shoulderY/mag3(shoulderY);
shoulderX=shoulderX/mag3(shoulderX);
shoulderZ=shoulderZ/mag3(shoulderZ);
shouldE=[shoulderX;shoulderY;shoulderZ];

% Determining glenohumeral center of rotation:
% METHOD 1: Translation from Acromioclavicular marker
% Determine shoulder coordinate system
% Translate AC marker a fixed distance along the shoulder�s Y-axis

% use this method for now.... assume that our glenoid marker is at the
% correct Y and Z locations.  Just need to take 
% X coordinate from the shoulder origin
gleno_center=[acromion(1),acromion(2),mean_pos(20)]; %this is also the UA origin

% Upper Arm (UA)
% 2.3.5. Humerus (2nd option) coordinate system�
% Xh2Yh2Zh2
% Oh2: The origin coincident with GH.

% Yh2: The line connecting GH and the midpoint of
% EL and EM, pointing to GH.
elb_axis=-mean_pos(24:26)+mean_pos(21:23); % lateral to medial
elb_center=mean_pos(24:26)+0.5*elb_axis;
UAY=gleno_center-elb_center;% the Yh2 axis
    wrist_axis=-mean_pos(27:29)+mean_pos(30:32); % ulnar to radial
    wrist_center=mean_pos(27:29)+0.5*wrist_axis;
    UlnStylPro=mean_pos(27:29)+10*wrist_axis/mag3(wrist_axis);
    % true location of ulnar styloid proc. 
    % is 10 mm in from the marker location, in the direction of the wrist axis
    LApY=elb_center-UlnStylPro;  % this is the Yf-axis, from uln TO elb
    
    
%%%%%%%%%%%%%%%%%%% The rest of these calculations only apply for the 
% calibration file with 90 deg. elbow flexion, arm by side.
if calibration_flag==1
    % Zh2: The line perpendicular to the plane formed by
    % Yh2 and Yf (see Section 2.3.6), pointing to the
    % right.
    UAZcalib=-cross(UAY,LApY);  %UAY points up along humerus, LApY points horiz along forearm. 
    % the sign on this needs to be flipped to point out from medial to
    % lateral.  This should be similar to the elb_axis now.  
    
% Xh2: The common line perpendicular to the Zh2- and
% Yh2-axis, pointing forward.

% Note 1: The second definition of humerus coordinate
% system is motivated by the high error sensitivity of the
% direction connecting EL and EM due to the short
% distance between them. Since it cannot be assured that
% the Zh2-axis is equal to the joint rotation axis, its
% orientation depends on the position of the upper arm
% and forearm as well as the forearm orientation (Wang,
% 1996). Therefore, by definition, the Zh2-axis is taken
% with the elbow flexed 90deg in the sagittal plane and the
% forearm fullypro nated.
    UAZcalib=UAZcalib/mag3(UAZcalib);
    % ok this is the coord system from the calibration
    % trial.  The only thing that would be different with the actual trials
    % is that we'd want to use the calibration-defined z-axis, since this
    % is theoretically more reliable than using the elbow axis, as far as
    % defining a rotation of the joint.  Let's do this by just grabbing the
    % elbow axis and calculating the difference between that and the actual
    % Z axis and then we can use that as an offset.
    UAZ_offset=UAZcalib - elb_axis/mag3(elb_axis);
else
    UAZ_offset=[-0.1601   -0.1286    0.0411]; %this is just the offset from one subject.  seems fairly typical.
end

UAZ=-elb_axis/mag3(elb_axis) -UAZ_offset; % now for non-calibration files we can just
% use the elbow axis, but we correct for it using this offset, since this
% is the more reliable measure of the true elbow axis of bending.
UAX=cross(UAY,UAZ);
UAX=UAX/mag3(UAX)
UAY=UAY/mag3(UAY)
UAZ=UAZ/mag3(UAZ)
UAE=[UAX;UAY;UAZ]

% Ok pretty sure this is correct according to the ISB definitions now
% 3/23/09

% Lower arm (distal)
% �	Y-vector from ulnar styloid to elbow center
% �	X-vector pointinng towards palm-side
% �	Z-vector pointing from ulna towards radius (roughly), perp to both
LAY=LApY; %previously defined Yf
LAX=cross(LAY,wrist_axis);
LAZ=cross(LAX,LAY);
% normalize to unit vectors
LAY=LAY/mag3(LAY);
LAX=LAX/mag3(LAX);
LAZ=LAZ/mag3(LAZ);
LAE=[LAX;LAY;LAZ];
% ok this is correct according to ISB definitions now 3/23/09
LA_origin=UlnStylPro; % ulnar styloid, origin for coord system
% Hand -  
% �	Y-vector points from 2nd metacarpal marker towards wrist center
% �	X-vector points palmar, perpindicular to wrist axis and Y vector
% �	Z-vector is perpindicular to both, pointing roughly from ulnar to radial

% mean_pos(33:35) is the 3rd metacarpal marker, which is 10 mm from the
% surface of the skin, and approx. 25 mm from the actual center of the bone
% problem is, we can't correct for that offset right now because we don't
% know which direction it's in, so we will just have to leave the
% correction as an extension angle offset.

handY=wrist_center-mean_pos(33:35); 
handX=cross(handY,wrist_axis);
handZ=cross(handX,handY);
% normalize to unit vectors
handY=handY/mag3(handY);
handX=handX/mag3(handX);
handZ=handZ/mag3(handZ);
handE=[handX;handY;handZ];

hand_origin=mean_pos(33:35); % hand origin at 2nd metacarpal for now.

% Calculate the euler angles for the shoulder (upper arm relative to trunk).
% Know that trunkE is in global coords, so E(trunk)=trunkE*I
% UAE (global), so E(UA)=UAE*I
% Then E(UA) = UA_about_trunk*E(trunk) = UA_about_trunk*trunkE*I
% mutliply both sides by inv(trunkE)
% to get UA_about_trunk = UAE*trunk_inv

% to get trunk kinematics we need to reference the trunk global frame,
% which is the matrix [0 1 0; 0 0 1; 1 0 0]
globalE=[-1 0 0; 0 0 1; 0 1 0]; %this is simply a reflection of how our subjects were positioned relative to global
% globalE=[1 0 0; 0 0 1; 0 -1 0]; % change for points defined in pelvis frame
% globalE=[0 1 0; 0 0 1; 1 0 0];
glob_inv=inv(globalE);
trunk_about_glob=trunkE*glob_inv;
trunk_about_glob=normalize(trunk_about_glob);

% % Method 1: find euler angles
[tr_a,tr_b]=rtocarda(trunk_about_glob,1,3,2);
if calibration_flag==1
    % this is the 90 deg. elbow flexion calibration file we're reading, so
    % use it to create offsets for elbow flexion, trunk states, and wrist
    % states
    tr_offsets=tr_a;
    % it's possible that the shoulder isn't exactly unflexed/abducted here,
    % so just use it to determine internal/external rotation offset.  
end
tr_a=tr_a-tr_offsets;

% calculate euler angles for the shoulder
trunkE
trunk_inv=inv(trunkE);
UAE_inv=inv(UAE);
% normalize to ensure each has a length of one.
UA_about_trunk=UAE*inv(trunkE);

UA_about_trunk

UA_about_trunk=normalize(UA_about_trunk);
% Method 1: euler angles (ISB recommendation)

[sh_a,sh_b]=rtocarda(UA_about_trunk,2,1,2);

%disp('SHOULDER')
%disp('exernal rotation,  elevation angle, plane of elevation')
%disp(sh_a)
if calibration_flag==2
        % this is the 90 deg. shoulder abduction file so ideally, we should
        % have euler angles of [xx,90,0] (xx is somewhere close to 0)
        sh_offsets(3)=sh_a(3);
        sh_offsets(2)=sh_a(2)-90;  
        disp('abduction calibration shoulder')
end
%%  for the 90 deg. elbow flexion, the humerus internal rotation should be
%%  darn close to zero.  Set it equal to zero for now.  
if calibration_flag==1
    sh_offsets(1)=sh_a(1);
    disp('no calibration shoulder')
end
%sh_offsets
%sh_a=sh_a-sh_offsets;
% flip the sign on the third number to make the plane of elevation correct
% - right now it's negative
%sh_a(3)=-sh_a(3);
% check sh_a(1) -- the external rotation angle -- to ensure congruancy.  It
% tends to get a sign flip around 180.  Take the diff.  If the diff is
% greater than 50, we need to take all of the values that are in that
% flipped region and add/subtract 180 from them to get the right value.

    
% disp(strcat(num2str(sh_a(1)),'          ,',num2str(sh_a(2)),'             ,',num2str(sh_a(3))))
% calculate the euler angles for the elbow.
LA_about_UA=LAE*UAE_inv;
LA_about_UA=normalize(LA_about_UA);
[elb_a,elb_b]=rtocarda(LA_about_UA,3,1,2);
% elbow doesn't seem to have any problems with euler angle discontinuities,
% so leave it alone

% disp('ELBOW')
% disp('flexion, pro/supination, (ignore)')
% disp(elb_a)

%%%%%  2-24-2011 Elbow flexion angle is goofy.  Try calculating using
%%%%%  shoulder, wrist, elbow centers of rotation and just assuming that
%%%%%  there is no ab/adduction and that int/ext rotation is negligable for
%%%%%  the purposes of elbow flexion.  Since we use this measure to divide
%%%%%  propulsion cycles, we need something a bit less error prone.
%  UAY and LAY are elb-to-sh, and wrist-to-elb long axes.  dot product will
%  give the included angle.  a dot b = cos(theta)*mag(a)*mag(b).  here
%  magnitudes are already set to 1.
elbowdot=dot(LAY,UAY);
elb_a(1)=acos(elbowdot)*180/pi;  % use this as the elbow flexion value for now.

% calculate euler angles for the wrist
LA_inv=inv(LAE);
hand_about_LA=handE*LA_inv;
hand_about_LA=normalize(hand_about_LA);
[wrist_a,wrist_b]=rtocarda(hand_about_LA,3,1,2);
% wrist has problems with euler angle discontinuities.  
if wrist_a(1)<=-90
    wrist_a(1)=wrist_a(1)+180;
    wrist_a(3)=wrist_a(3)+180;
end
if wrist_a(1)>=180
    wrist_a(1)=wrist_a(1)-180;
    wrist_a(3)=wrist_a(3)-180;
end    
% disp('WRIST')
% disp('extension, ulnar deviation, (ignore)')
% % here is where we need to correct for the offset of the hand marker by
% % subtracting the offset (~18.5 degrees) from the wrist extension

wrist_a(1:2)= wrist_a(1:2) - wrist_offsets;  %default wrist offset is 18.5

% create the matrix of output data.  this includes the following:
% Time, trunk_flexion, tr_lat_flex_left, tr_rotation_CCW, sh exernal rotation,  
% sh elevation angle, sh plane of elevation, elb_flex, wrist_ext, wrist_rad_dev
outputdata=[time,tr_a,sh_a,elb_a(1),wrist_a];
