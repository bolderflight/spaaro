function flightplan = load_waypoint (flightplan, waypoint_file)
fid = fopen(waypoint_file);
j=1;
tline = fgetl(fid);
while ischar(tline)
    line(j,:) = convertCharsToStrings(tline);
    tline = fgetl(fid);
    j=j+1;
end
waypoint_str = line(3:end,:);

for i = 1:size(waypoint_str,1)
    waypoint(i,:) = strsplit(waypoint_str(i,1),'\t');
end

waypoint = str2double(waypoint);
for i=1:size(waypoint,1)
    flightplan(i).autocontinue = waypoint(i,12);
    flightplan(i).frame = waypoint (i,3);
    flightplan(i).cmd = waypoint (i,4);
    flightplan(i).param1 = waypoint (i,5);
    flightplan(i).param2 = waypoint (i,6);
    flightplan(i).param3 = waypoint (i,7);
    flightplan(i).param4 = waypoint (i,8);
    flightplan(i).x = waypoint(i,9);
    flightplan(i).y = waypoint(i,10);
    flightplan(i).z = waypoint(i,11);
end
end