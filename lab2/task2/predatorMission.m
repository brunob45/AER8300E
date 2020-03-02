%this is the predator simulation scenario

clc
clear all
close all

%%
app = actxserver('STK11.application');
root = app.Personality2; 

scenario = root.Children.New('eScenario','MATLAB_PredatorMission');
scenario.SetTimePeriod('24 Feb 2012 16:00:00.000','25 Feb 2012 16:00:00.000');
scenario.StartTime = '24 Feb 2012 16:00:00.000';
scenario.StopTime = '25 Feb 2012 16:00:00.000';
root.ExecuteCommand('Animate * Reset');

%%
facility = scenario.Children.New('eFacility','GroundStation');
facility.Position.AssignGeodetic(36.1457,-114.5946,0);
satellite = scenario.Children.New('eSatellite','GeoSat'); 


keplerian = satellite.Propagator.InitialState.Representation.ConvertTo('eOrbitStateClassical');
keplerian.SizeShapeType = 'eSizeShapeAltitude';
keplerian.LocationType = 'eLocationTrueAnomaly';
keplerian.Orientation.AscNodeType = 'eAscNodeLAN';
keplerian.SizeShape.PerigeeAltitude = 35788.1;
keplerian.SizeShape.ApogeeAltitude = 35788.1;
keplerian.Orientation.Inclination = 0;
keplerian.Orientation.ArgOfPerigee = 0;
keplerian.Orientation.AscNode.Value = 245;
keplerian.Location.Value = 180;
satellite.Propagator.InitialState.Representation.Assign(keplerian);
satellite.Propagator.Propagate;

%% Get the path to the STK install directory
installDirectory = root.ExecuteCommand('GetDirectory / STKHome').Item(0);
aircraft = scenario.Children.New('eAircraft','Predator');
model = aircraft.VO.Model;
model.ModelData.Filename = [installDirectory 'STKData\VO\Models\Air\rq-1a_predator.mdl']; 

aircraft.SetRouteType('ePropagatorGreatArc');
route = aircraft.Route;
route.Method = 'eDetermineTimeAccFromVel';
route.SetAltitudeRefType('eWayPtAltRefMSL'); 

waypoint = route.Waypoints.Add();
waypoint.Latitude = 46.098;
waypoint.Longitude = -122.0823;
waypoint.Altitude = 4.5; % km
waypoint.Speed = .075; % km/sec
waypoint.TurnRadius = 0; % km

waypoint = route.Waypoints.Add();
waypoint.Latitude = 46.269;
waypoint.Longitude = -122.192;
waypoint.Altitude = 4.5; % km
waypoint.Speed = .075; % km/sec
waypoint.TurnRadius = 0.25; % km

waypoint = route.Waypoints.Add();
waypoint.Latitude = 46.251;
waypoint.Longitude = -122.248;
waypoint.Altitude = 4.5; % km
waypoint.Speed = .075; % km/sec
waypoint.TurnRadius = 0.25; % km

waypoint = route.Waypoints.Add();
waypoint.Latitude = 46.076;
waypoint.Longitude = -122.131;
waypoint.Altitude = 4.5; % km
waypoint.Speed = .075; % km/sec
waypoint.TurnRadius = 0; % km

route.Propagate; 

%% Adding Terrain and Imagery and Setting Camera View
%Instantiates the SceneManager object for use
manager = scenario.SceneManager;
%Adds Terrain in for analysis
cmd = ['Terrain * Add Type PDTT File "' installDirectory 'STKData\VO\Textures\St Helens.pdtt"'];
root.ExecuteCommand(cmd);
%Visually implements the terrain to our 3D Graphics
earthTerrain = manager.Scenes.Item(0).CentralBodies.Earth.Terrain;
terrainTile = earthTerrain.AddUriString([installDirectory 'STKData\VO\Textures\St Helens.pdtt']);
terrain.UseTerrain = true;

%Visually implements the imagery to our 3D Graphics
earthImagery = manager.Scenes.Item(0).CentralBodies.Earth.Imagery;
imageryTile = earthImagery.AddUriString([installDirectory 'STKData\VO\Textures\St Helens.jp2']);
extentImagery = imageryTile.Extent;
disp('Imagery boundaries: ');
disp(['LatMin: ' num2str(extentImagery{1}) ' LatMax: ' num2str(extentImagery{3})]);
disp(['LonMin: ' num2str(extentImagery{2}) ' LonMax: ' num2str(extentImagery{4})]);
%Enables 3D Graphics Window label declutter
root.ExecuteCommand('VO * Declutter Enable On');

axes = aircraft.Vgt.Axes.Item('TopoCentric');
point = aircraft.vgt.Points.Item('Center');
offset = {-5.1575; -0.75; 6.0};
upDirection = {-0.0906; -0.6908; 0.7173};
manager = scenario.SceneManager;
camera = manager.scenes.Item(0).Camera;
camera.ViewOffsetWithUpAxis(axes , point, offset, upDirection);
camera.ConstrainedUpAxis = 'eStkGraphicsConstrainedUpAxisZ';
camera.FieldOfView = 45;
camera.LockViewDirection = false;
manager.Render;

%% Creating Targets with Random Locations Using Automation
nTargets = 3;
targetlocations_lat = 46.1991 - .035 + .07*rand(1,nTargets);
targetlocations_long = -122.1864 - .035 + .07*rand(1,nTargets);
for i = 1:nTargets
    tname = ['Target' num2str(i)];
    target(i) = scenario.Children.New('eTarget',tname);
    target(i).Position.AssignGeodetic(targetlocations_lat(1,i),targetlocations_long(1,i),0);
    target(i).UseTerrain = true;
    target(i).SetAzElMask('eTerrainData',0);
    root.ExecuteCommand(['SetConstraint */Target/' tname ' AzElMask On']);
end

%% Retrieving Data Providers

root.UnitPreferences.Item('DateFormat').SetCurrentUnit('EpSec');
llaFixed = aircraft.DataProviders.Item('LLA State').Group.Item('Fixed');
aircraftPosDP = llaFixed.Exec(scenario.StartTime,scenario.StopTime,30);
aircraftLat = cell2mat(aircraftPosDP.DataSets.GetDataSetByName('Lat').GetValues);
aircraftLon = cell2mat(aircraftPosDP.DataSets.GetDataSetByName('Lon').GetValues);
aircraftAlt = cell2mat(aircraftPosDP.DataSets.GetDataSetByName('Alt').GetValues);
disp([aircraftLat aircraftLon aircraftAlt]) 

%% Adding a Sensor and Defining Pointing
sensor = aircraft.Children.New('eSensor', 'Targeted');
pattern1 = sensor.Pattern;
pattern1.ConeAngle = 5;
sensor.SetPointingType('eSnPtTargeted');
pointing1 = sensor.Pointing;
pointing1.Targets.AddObject(target(1)); 

%% Targeting the Sensor at the Closest Target
%% Computing Access and Chain Access
access = satellite.GetAccessToObject(facility);
access.ComputeAccess;
accessDP = access.DataProviders.Item('Access Data').Exec(scenario.StartTime,scenario.StopTime);
accessStartTimes = accessDP.DataSets.GetDataSetByName('Start Time').GetValues;
accessStopTimes = accessDP.DataSets.GetDataSetByName('Stop Time').GetValues;
llaFixed = satellite.DataProviders.Item('LLA State').Group.Item('Fixed');
satelliteDP = llaFixed.ExecElements(accessStartTimes{1},accessStopTimes{1},30,{'Time';'Alt'});
satellitealtitude = satelliteDP.DataSets.GetDataSetByName('Alt').GetValues;

chain = scenario.Children.New('eChain', 'SensorInfoNetwork');
chain.Objects.AddObject(target(1));
chain.Objects.AddObject(sensor);
chain.Objects.AddObject(aircraft);
chain.Objects.AddObject(satellite);
chain.Objects.AddObject(facility);
chain.ComputeAccess();

%% Animating the Scenario
path = [installDirectory 'STKData/VO/Textures/St Helens.jp2"'];
cmd = ['VO * ViewFromTo Normal From "' path];
root.ExecuteCommand(cmd);
root.ExecuteCommand('VO * View Zoom WindowID 1 FractionofCB -0.0015');
animation = scenario.Animation;
animation.AnimStepValue = 0.5;
root.ExecuteCommand('Animate * Start End');






















