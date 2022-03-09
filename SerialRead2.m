% Matlab Script to Read and Plot Serial Data
%
%           Version 4.0 (11/01/2018) H.C.
%           Version 5.0 (09/24/2020) H.C.
%           Version 6.0 (10/13/2021) H.C.
%           Version 7.0 (02/21/2022) H.C.
%

% make sure no residual serial object in Matlab workspace
if (exist('s1','var'))
    delete(s1);
    clear s1;
end

% clear all figures and variables in workspace
close all
clear data data_char status
delete(instrfind);

% modify data labels for the 3 signals here
Labels = {'Set point 1','Sensor 1','Set point 2', 'Sensor 2'};

try

% define serial object with matching com port and baud rate
% change com port number and/or baud rate if needed

% s1 = serialport("COM3",115200);       % Windows
s1 = serialport("/dev/cu.usbmodem144101",115200);        % MacOS

disp(' ');
disp('*** Serial Data Capture ***');
disp('*** Press the STOP button to end ***');
disp(' ');

status = getpinstatus(s1);
configureTerminator(s1,"LF");

% initialize variables
i = 1;

% create a STOP button on the figure window
hFig = figure(1);
hFig.Name = 'Serial Data Plot';
ax = axes(hFig);
ax.Units = 'normalized';
ax.Position = [0.125 0.15 0.775 0.775];
hButton = uicontrol(hFig,'Style','pushbutton');
hButton.String = 'STOP';
hButton.BackgroundColor = [1 0 0];
hButton.ForegroundColor = [1 1 1];
hButton.FontWeight = 'bold';
hButton.UserData = 0;

% here we define 3 data lines, add or substract lines if needed
h1 = animatedline ('Color','g');
h2 = animatedline ('Color','b');
h3 = animatedline ('Color','c');
h4 = animatedline ('Color','r');

title('Streaming Serial Data <Press the STOP button to end>')
xlabel('Time (sec)'),ylabel('Values'), grid
legend(Labels);

% get data from the serial object till the STOP button is pressed
while ( hButton.UserData == 0 )

    data_char{i} = readline(s1);
    
    if(~strcmp(data_char{i},''))
        data(i,:) = str2num(data_char{i});
    else
        break;
    end
    
    addpoints(h1, data(i,1), data(i,2));
    addpoints(h2, data(i,1), data(i,3));
    addpoints(h3, data(i,1), data(i,4));
    addpoints(h4, data(i,1), data(i,5));
    legend(Labels);

    drawnow limitrate
    hButton.Callback = ['hButton.UserData = 1;','disp(''Stopping data collection...'')'];
    i = i+1;
end

% drawnow;
fprintf(['*** Done. A total of ',num2str(i-1), ' datasets collected ***\n']);
disp(' ');

% re-plot the data once the data collection is done to get all figure features
figure(1)
plot(data(:,1), data(:,2),'g', data(:,1), data(:,3),'b', data(:,1), data(:,4),'c', data(:,1), data(:,5),'r')
title('Serial Data')
xlabel('Time (sec)'),ylabel('Values'), grid
legend(Labels);

dt = mean(diff(data(:,1)));
disp(['Sampling period (sec) = ', num2str(dt)])
disp(['Sampling frequency (Hz) = ', num2str(1/dt)])

% To disconnect the serial port object from the serial port
delete(s1);
clear s1;

catch ME
    warn_string = ["1. Check if Serial Port ID is set correctly.",...
        "2. Make sure Serial Monitor or Plotter is not running.",...
        "3. #define MATLAB_SERIAL_READ in Arduino code is enabled.",... 
        "4. Do not close the figure window while the program is running.",...
        "5. Hit a key on the keyboard while the real-time graph",...
        "     is active to exit the program properly."];   
    warndlg(warn_string, 'Serial Read Warning');
    disp(' ');
    disp('Program terminated abnormally!');
    disp(['Cause: ' ME.identifier]);
    if (exist('s1','var'))
        delete(s1);
        clear s1;
    end
end
% Use the command below to force clear any hidden com port if needed
% delete(instrfind);
%% plot circles

L1 = 0.1524; % The length of the first link
L2 = 0.1524; % The length of the second link

% assign data to proper variables
sp1 = data(:,2);
q1 = data(:,3);
sp2 = data(:,4);
q2 = data(:,5);

% reference circle
X = L1 * cos(sp1) + L2 * cos(sp1+sp2);
Y = L1 * sin(sp1) + L2 * sin(sp1+sp2);

% actual circle
x = L1 * cos(q1) + L2 * cos(q1+q2);
y = L1 * sin(q1) + L2 * sin(q1+q2);

figure(2)
plot(X, Y,'b',x, y,'r'), grid;
legend('Target','Actual');
axis equal
title('Trajectory Comparison'), xlabel('X'), ylabel('Y');

