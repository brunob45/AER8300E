
clc
clear all
close all

%%
app = actxserver('STK11.application');
root = app.Personality2; 

scenario = root.Children.New('eScenario','MATLAB_PredatorMission');
scenario.SetTimePeriod('19 Feb 2020 00:00:00.000','19 Feb 2021 00:00:00.000');
scenario.StartTime = '19 Feb 2020 00:00:00.000';
scenario.StopTime = '19 Feb 2021 00:00:00.000';
root.ExecuteCommand('Animate * Reset');

%% 














