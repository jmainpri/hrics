% Shell program to run the motion analysis code for upper extremity
% motion.  This program provides a loop so that all lines of the motion
% data can be analyzed to generate a continuous series of joint motions.
% Written by karen Troy 3/17/09
% Modified by Karen Troy 6/30/14 for Dmitry Berenson's lab

% Output is a tab-delimited file with one line per frame and each column is
% a variable

% First open and analyze the static trials so that we can determine the
% offset angles (ie 90 deg elbow flexion, shoulder abduction etc).
clear all;

tr_offsets=[0 0 0];
sh_offsets=[0 0 0];
elb_offset=0;
wrist_offsets=[0 0];
% wrist_offsets=[18.5 0]; %this is the angle made by the vectors that go from wrist-center to hand, and wrist-center to marker-center
static1yes=0;  %flag for static trial #1
static2yes=0;  %flag for static trial #2
calibration_flag=0;

%static1yes=input('Do you have a 90 deg. elbow flexion static trial? 1=yes 0=no ');
if static1yes==1
[input_file, pathname] = uigetfile('*.trc', 'Select the static trial with 90 deg. elbow flexion, pronated wrist');
open_name=strcat(pathname,input_file);
    [labels,x,data_all] = readTRCData(open_name,35,7);  %you need the m-file for this or will need to modify this input line
    numlines=size(data_all,1);
    % filter all of the data through a 6th order butterworth, cutoff freq
    % 5 Hz. -- filtfilt doubles the order of the filter.
    SF=10/(data_all(20,2)-data_all(10,2));  %sampling frequency, in Hz
    [b,a]=butter(3,5/(SF/2)); %the 5 Hz butterworth filter
    data_all=filtfilt(b,a,data_all);
    mean_pos=mean(data_all);
    % let the program know this is a calibration file
    % initialize offsets; these will be determined from the calibration
    calibration_flag=1;
    Upper_extremity_kinematics_for_Dmitry;  %this calls the program that actually calculates the inverse kinematics
end
calibration_flag=0;
%static2yes=input('Do you have a 90 deg. shoulder abduction static trial? 1=yes 0=no ');
if static2yes==1
[input_file, pathname] = uigetfile('*.trc', 'Select the static trial with 90 deg. shoulder abduction');
open_name=strcat(pathname,input_file);
    [labels,x,data_all] = readTRCData(open_name,35,7);
    numlines=size(data_all,1);
    mean_pos=mean(data_all);
    % let the program know this is a calibration file
    % initialize offsets; these will be determined from the calibration
    calibration_flag=2;
    Upper_extremity_kinematics_for_Dmitry;
end
if static2yes==0
    sh_offsets(2)=-10;  % set the offset to -10 for the shoulder if there is no calibration file for abduction
    % this value is arbitrary and based on our own marker set and subject
    % anatomy
end
calibration_flag=0; %reset the flag so that the data files aren't treated as calibration files.
    disp('trunk offsets');disp(tr_offsets);  %these are Euler angles that disply local coordinate systems in the global frame
    disp('shoulder offsets');disp(sh_offsets);
    disp('elbow offset');disp(elb_offset);
    disp('wrist offsets');disp(wrist_offsets);
% Now start the remaining analysis
%filename=input('What do you want to call your output data file? ','s');
filename='outputik'
% Create the output file header information
fid = fopen(filename,'w');
fprintf(fid,'Upper extremity biomechanics analysis; joint kinematics\n');
b=clock;
timenow=b(4:5);
fprintf(fid,strcat('Created at %2.0f:%-2.0f on .',date),timenow);
fprintf(fid,'\n');
fprintf(fid,'\n');
% Open the data file to analyze
% code to read raw .trc data files
% [input_file, pathname] = uigetfile('*.trc', 'Pick a data file');
input_file = 'RERC1A4_6_short.trc'
pathname = './'
open_name=strcat(pathname,input_file);

[labels,x,data_all] = readTRCData(open_name,35,6);
%% include the line below if you want to ignore the first n rows of
%% data
%     data_all(1:4000,:)=[];  %deletes the first 4000 rows
%     numlines=size(data_all,1);
%     data_all(11600:numlines,:)=[];  % deletes the rows after frame 11600
    numlines=size(data_all,1);    
% Finish the output file headers now    
fprintf(fid,'Data_file:\t %s\n',input_file);
fprintf(fid,'Time\t tr_lat_flex_left\t tr_extension\t tr_rot_left\t sh_int_rot\t sh_elevation\t sh_plane_of_elevation\t elb_flex\t wrist_ext\t wrist_uln_dev\n');
% the directions in the labels (ie tr_lat_flex_left) indicate that a
% rotation to the left is positive
% create an array in which to put the data so we can plot it
kinematics=0:10;
for i=1:numlines
    mean_pos=data_all(i,:)
    disp(['ik for line : ' num2str(i)])
    Upper_extremity_kinematics_for_Dmitry;
    kinematics=[kinematics;outputdata];
    fprintf(fid,'%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\n6.4f\n',outputdata);
end
fclose(fid);
disp(['save to ' filename '.csv']);
csvwrite( [filename '.csv'], kinematics );

