function run_one_ik()
% Shell program to run the motion analysis code for upper extremity
% motion.  This program provides a loop so that all lines of the motion
% data can be analyzed to generate a continuous series of joint motions.
% Written by karen Troy 3/17/09
% Modified by Karen Troy 6/30/14 for Dmitry Berenson's lab
clear all;

current_path = pwd;
current_path = [current_path '/matlab/'];
addpath(current_path);

tr_offsets=[0 0 0];
sh_offsets=[0 0 0];
elb_offset=0;
wrist_offsets=[0 0];
% wrist_offsets=[18.5 0]; %this is the angle made by the vectors that go from wrist-center to hand, and wrist-center to marker-center
static1yes=0;  %flag for static trial #1
static2yes=0;  %flag for static trial #2
calibration_flag=0; %reset the flag so that the data files aren't treated as calibration files.

% Set input file name
disp('load file');
input_file = 'positions_fixed.csv';
pathname = current_path;
[data_all] = readViconData( strcat( pathname, input_file ), 1 );


% Perform IK
disp('perform ik');
kinematics=0:10;
mean_pos=data_all(1,:);
Upper_extremity_kinematics_for_Dmitry;
kinematics=[kinematics;outputdata];

% Set out put file name
disp('save to file');
filename='outputik'
disp(['save to ' filename '.csv']);
csvwrite( [filename '.csv'], kinematics );
