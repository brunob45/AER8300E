%this is the predator simulation scenario

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

%% Montreal
montreal = scenario.Children.New('eFacility','Montreal');
montreal.Position.AssignGeodetic(45.5889,-73.5616,0);

%% Satellite1
sat1 = scenario.Children.New('eSatellite','SunSat1');
root.ExecuteCommand('OrbitWizard */Satellite/SunSat1 SunSynchronous Altitude 901000 LocalTimeAscNode 20:00:00.000');

%% Satellite2
sat2 = scenario.Children.New('eSatellite','SunSat2');
root.ExecuteCommand('OrbitWizard */Satellite/SunSat2 SunSynchronous Altitude 1688000 LocalTimeAscNode 18:15:00.000');

%% Satellite3
sat3 = scenario.Children.New('eSatellite','SunSat3');
root.ExecuteCommand('OrbitWizard */Satellite/SunSat3 SunSynchronous Altitude 1200000 LocalTimeAscNode 12:00:00.000');








